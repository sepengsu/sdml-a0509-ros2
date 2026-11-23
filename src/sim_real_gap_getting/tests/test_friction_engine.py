"""Force/sign/history tests against the original code and MuJoCo itself."""
from copy import deepcopy
from pathlib import Path
import sys
import tempfile
import unittest

import mujoco as mj
import numpy as np

ROOT=Path(__file__).resolve().parents[3]
for name in ("sim_real_gap_getting","a0509_dynamics_experiment","mdpg_1s1_description"):
    sys.path.insert(0,str(ROOT/"src"/name))
from sim_real_gap_getting.friction_engine.common import PACKAGE,build_baseline,load_config,original_functions
from sim_real_gap_getting.friction_engine.laws import FrictionState,configure_model,decompose,inverse_at,parameters,resistance,scenarios
from sim_real_gap_getting.friction_engine import dynamic,models,runtime
from unittest.mock import patch
import xml.etree.ElementTree as ET


class FrictionEngineTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.temp=tempfile.TemporaryDirectory()
        cls.root=Path(cls.temp.name)
        cls.config=load_config(PACKAGE/"config.json")
        cls.model=build_baseline("bare_p0",cls.root/"base.xml",cls.config)
        cls.q=np.array((-.5,-.1,.3,0.,-.2,0.))
        cls.v=np.array((.1,-.1,.08,-.08,.03,-.03))
        cls.a=np.array((.01,-.01,.02,-.02,.03,-.03))
        cls.baseline=scenarios(cls.config)[0]

    @classmethod
    def tearDownClass(cls): cls.temp.cleanup()

    def case(self,name): return next(s for s in scenarios(self.config) if s["name"]==name)

    def test_original_config_and_all_fifteen_scenarios_are_pinned(self):
        self.assertEqual(len(scenarios(self.config,False)),15)
        np.testing.assert_allclose(parameters(self.case("coulomb_high"),self.config)["fc_nm"],[.6,.525,.375,.225,.18,.12],atol=1e-15)

    def test_portable_lock_blocks_a_second_writer(self):
        folder=self.root/"lock"; folder.mkdir(exist_ok=True)
        with runtime.run_lock(folder):
            with self.assertRaises(OSError):
                with runtime.run_lock(folder):
                    self.fail("Acquired a conflicting writer lock")
        with runtime.run_lock(folder):
            pass

    def test_older_model_builders_preserve_inertia_and_collision(self):
        def builder(path):
            Path(path).write_text('<mujoco><asset><mesh name="ag145_visual_mesh" file="unused.stl"/></asset>'
                '<worldbody><body name="tool"><inertial mass="1.3" pos="0 0 0" diaginertia="1 1 1"/>'
                '<geom name="ag145_body_collision" type="box" size="1 1 1" density="0"/>'
                '<geom name="ag145_body_visual" mesh="ag145_visual_mesh" type="mesh" density="0" contype="0"/>'
                '</body></worldbody></mujoco>')
        path=self.root/"headless.xml"
        models._headless(builder,path)
        root=ET.parse(path).getroot()
        self.assertEqual(root.find(".//inertial").get("mass"),"1.3")
        self.assertIsNotNone(root.find(".//geom[@name='ag145_body_collision']"))
        self.assertIsNone(root.find(".//geom[@name='ag145_body_visual']"))
        self.assertIsNone(root.find(".//mesh"))

    def test_tool_selector_limits_plan_to_256_rollouts(self):
        from sim_real_gap_getting.friction_engine import cli
        from contextlib import redirect_stdout
        from io import StringIO
        for tool in ("none","ag145","mdpg_1s1"):
            with patch.object(cli,"identity",return_value=("test",{})),redirect_stdout(StringIO()) as captured:
                result=cli.main(["--source",str(self.root/"source"),"--legacy-speed-root",str(self.root/"old"),
                                 "--output",str(self.root/"new"),"--tool",tool])
            self.assertEqual(result,0)
            self.assertIn("selected scope 256",captured.getvalue())

    def test_viscous_native_damping_is_added_once(self):
        configure_model(self.model,self.baseline,self.config)
        data=mj.MjData(self.model)
        base,_=inverse_at(self.model,data,self.q,self.v,self.a,background=self.config["background_damping_nm_s_rad"])
        scenario=self.case("viscous_high"); configure_model(self.model,scenario,self.config)
        result,parts=inverse_at(self.model,data,self.q,self.v,self.a,background=self.config["background_damping_nm_s_rad"])
        expected=np.asarray(self.config["legacy"]["viscous_base_nm_s_rad"])*1.5*self.v
        np.testing.assert_allclose(result-base,expected,atol=1e-9)
        np.testing.assert_allclose(parts["qfrc_additional_friction"],-expected,atol=1e-9)
        np.testing.assert_array_equal(FrictionState(scenario,self.config).advance(0.,self.v),np.zeros(6))

    def test_coulomb_is_a_constraint_not_an_added_passive_force(self):
        configure_model(self.model,self.baseline,self.config)
        data=mj.MjData(self.model)
        base,bparts=inverse_at(self.model,data,self.q,self.v,self.a,background=self.config["background_damping_nm_s_rad"])
        scenario=self.case("coulomb_nominal"); configure_model(self.model,scenario,self.config)
        result,parts=inverse_at(self.model,data,self.q,self.v,self.a,background=self.config["background_damping_nm_s_rad"])
        np.testing.assert_allclose(parts["qfrc_passive"],bparts["qfrc_passive"],atol=1e-9)
        np.testing.assert_allclose(result-base,-parts["qfrc_native_dry"],atol=1e-8)
        self.assertTrue(np.all(np.abs(result-base)<=np.asarray(self.config["legacy"]["friction_base_nm"])+5e-5))
        np.testing.assert_array_equal(FrictionState(scenario,self.config).advance(0.,self.v),np.zeros(6))

    def test_external_friction_motor_inverse_is_corrected_once(self):
        configure_model(self.model,self.baseline,self.config)
        data=mj.MjData(self.model)
        base,_=inverse_at(self.model,data,self.q,self.v,self.a,background=self.config["background_damping_nm_s_rad"])
        scenario=self.case("stribeck_high"); configure_model(self.model,scenario,self.config)
        external=-resistance(self.v,scenario,self.config)
        native,parts=inverse_at(self.model,data,self.q,self.v,self.a,external,self.config["background_damping_nm_s_rad"])
        np.testing.assert_allclose(native,base,atol=1e-9)
        np.testing.assert_allclose(native-external-base,-external,atol=1e-9)
        np.testing.assert_allclose(parts["qfrc_additional_friction"],external,atol=1e-9)
        np.testing.assert_array_equal(self.model.dof_frictionloss,np.zeros(6))
        np.testing.assert_array_equal(self.model.dof_damping,self.config["background_damping_nm_s_rad"])

    def test_rbd_and_all_force_channels_balance_in_forward_dynamics(self):
        for scenario in scenarios(self.config):
            configure_model(self.model,scenario,self.config)
            data=mj.MjData(self.model); data.qpos[:]=self.q; data.qvel[:]=self.v
            external=FrictionState(scenario,self.config).advance(0.,self.v)
            data.qfrc_applied[:]=external
            data.ctrl[:]=np.array((80.,-70.,60.,-20.,20.,-10.))
            mj.mj_forward(self.model,data)
            parts=decompose(self.model,data,self.config["background_damping_nm_s_rad"])
            np.testing.assert_allclose(parts["force_balance_residual_nm"],0,atol=1e-7)
            inverse=mj.MjData(self.model)
            raw,_=inverse_at(self.model,inverse,data.qpos,data.qvel,data.qacc,external,self.config["background_damping_nm_s_rad"])
            np.testing.assert_allclose(raw-external,data.qfrc_actuator,atol=1e-7)

    def test_static_laws_match_original_functions(self):
        v=np.array([[-.1]*6,[0.]*6,[.1]*6])
        old=self.config["legacy"]; reference=original_functions()
        for name,func in (("stribeck","_smooth_friction"),("asymmetric","_asymmetric_friction")):
            for level,scale in old["levels"].items():
                expected=reference[func](v,np.asarray(old["friction_base_nm"])*scale,
                                        np.asarray(old["viscous_base_nm_s_rad"])*scale,old)
                np.testing.assert_allclose(resistance(v,self.case(f"{name}_{level}"),self.config),expected,atol=1e-14)

    def test_lugre_matches_original_zero_state_full_history(self):
        t=np.arange(101)*.1
        v=np.sin(t[:,None]+np.arange(6)[None,:])*.08
        old=self.config["legacy"]
        for level,scale in old["levels"].items():
            expected=original_functions()["_lugre_friction"](t,v,np.asarray(old["friction_base_nm"])*scale,
                           np.asarray(old["viscous_base_nm_s_rad"])*scale,old)
            state=FrictionState(self.case("lugre_"+level),self.config)
            actual=np.array([-state.advance(float(stamp),velocity) for stamp,velocity in zip(t,v)])
            np.testing.assert_allclose(actual,expected,atol=1e-14)

    def test_lugre_cannot_advance_twice_in_one_physics_step(self):
        state=FrictionState(self.case("lugre_nominal"),self.config)
        state.advance(0.,self.v); state.advance(.002,self.v)
        before=state.z.copy()
        with self.assertRaises(ValueError): state.advance(.002,self.v)
        np.testing.assert_array_equal(state.z,before)

    def test_coulomb_zero_velocity_force_remains_bounded(self):
        scenario=self.case("coulomb_high"); configure_model(self.model,scenario,self.config)
        raw,parts=inverse_at(self.model,mj.MjData(self.model),self.q,np.zeros(6),np.zeros(6),background=self.config["background_damping_nm_s_rad"])
        self.assertTrue(np.all(np.abs(parts["qfrc_native_dry"])<=np.asarray(parameters(scenario,self.config)["fc_nm"])+1e-6))

    def test_dynamic_executes_all_force_backends_and_records_raw_channels(self):
        config=deepcopy(self.config)
        identity="unit-test"
        for scenario in scenarios(config):
            job={"condition":"bare_p0","trajectory":"T15","scenario":scenario,"phase":"pilot","duration_s":.04,"split":"train"}
            output=self.root/"dynamic"
            meta=dynamic.run_trial(job,output,config,identity)
            self.assertTrue(meta["closed_loop_rollouts"])
            self.assertEqual(meta["physics_steps"],20)
            with np.load(dynamic.folder_for(output,job)/"samples.npz") as data:
                self.assertEqual(data["qfrc_passive"].shape,(5,6))
                self.assertEqual(data["qfrc_additional_friction"].shape,(5,6))
                np.testing.assert_allclose(data["required_motor_inverse_nm"],data["tau_actuator_nm"],atol=1e-6)


if __name__=="__main__": unittest.main()
