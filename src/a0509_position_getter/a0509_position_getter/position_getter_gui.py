import threading
import time
import tkinter as tk
from tkinter import ttk, messagebox

def _format_pos(pos):
    if pos is None: return "-"
    return "%.2f" % pos

def run_gui(node, root):
    root.title("A0509 Robot Control Panel")
    root.geometry("600x850") # 세로로 긴 형태
    
    # 스타일 설정
    style = ttk.Style()
    style.configure("Header.TLabel", font=('Helvetica', 12, 'bold'))
    style.configure("Value.TLabel", background="white", anchor="center", font=('Consolas', 10))
    style.configure("Selected.TButton", background="#2980b9", foreground="white") # 선택된 축 강조

    # --- [상단] Robot Mode ---
    f_mode = ttk.LabelFrame(root, text="Robot Mode", padding=10)
    f_mode.pack(fill=tk.X, padx=10, pady=5)

    def on_set_mode(m):
        threading.Thread(target=lambda: node.set_robot_mode(m, lambda s, msg: 
            root.after(0, lambda: messagebox.showinfo("Mode", msg) if s else messagebox.showerror("Error", msg))), 
            daemon=True).start()

    ttk.Button(f_mode, text="Manual Mode", command=lambda: on_set_mode(0)).pack(side=tk.LEFT, expand=True, fill=tk.X, padx=5)
    ttk.Button(f_mode, text="Auto Mode", command=lambda: on_set_mode(1)).pack(side=tk.LEFT, expand=True, fill=tk.X, padx=5)

    def on_home():
        def _run():
            def done(ok):
                root.after(0, lambda: (
                    messagebox.showinfo("Home", "Move home 성공") if ok
                    else messagebox.showerror("Home", "Move home 실패")
                ))
            node.go_home(done)
        threading.Thread(target=_run, daemon=True).start()

        # end on_home

    ttk.Button(f_mode, text="Go Home", command=on_home).pack(side=tk.LEFT, expand=True, fill=tk.X, padx=5)

    # --- Solution Space (현재 역기구학 해 공간 0~7) ---
    f_sol = ttk.LabelFrame(root, text="Solution Space", padding=10)
    f_sol.pack(fill=tk.X, padx=10, pady=5)
    ttk.Label(f_sol, text="현재 로봇의 Solution Space (0~7):", style="Header.TLabel").pack(side=tk.LEFT, padx=(0, 8))
    sol_space_label = ttk.Label(f_sol, text="—", width=6, style="Value.TLabel", relief="sunken")
    sol_space_label.pack(side=tk.LEFT)

    # --- [중간] Jogging Panel (이미지 스타일) ---
    f_jog = ttk.LabelFrame(root, text="Jogging Control", padding=10)
    f_jog.pack(fill=tk.BOTH, expand=False, padx=10, pady=5)

    # 상태 변수
    selected_axis = tk.IntVar(value=0) # 0-5: Joint, 6-11: Task
    joint_labels = []
    task_labels = []

    # 레이아웃 헤더
    ttk.Label(f_jog, text="Joint", style="Header.TLabel").grid(row=0, column=0, columnspan=2, pady=5)
    ttk.Label(f_jog, text="Task", style="Header.TLabel").grid(row=0, column=2, columnspan=2, pady=5)

    # 축 선택 및 표시부 (6행 구성)
    axis_names = ["J1", "J2", "J3", "J4", "J5", "J6", "X", "Y", "Z", "Rx", "Ry", "Rz"]
    
    for i in range(6):
        # Joint 측 (왼쪽)
        btn_j = tk.Button(f_jog, text=axis_names[i], width=5, 
                          command=lambda idx=i: selected_axis.set(idx))
        btn_j.grid(row=i+1, column=0, padx=2, pady=2, sticky="nsew")
        lab_j = ttk.Label(f_jog, text="0.00", width=10, style="Value.TLabel", relief="sunken")
        lab_j.grid(row=i+1, column=1, padx=2, pady=2, sticky="nsew")
        joint_labels.append(lab_j)

        # Task 측 (오른쪽)
        btn_t = tk.Button(f_jog, text=axis_names[i+6], width=5, 
                          command=lambda idx=i+6: selected_axis.set(idx))
        btn_t.grid(row=i+1, column=2, padx=2, pady=2, sticky="nsew")
        lab_t = ttk.Label(f_jog, text="0.00", width=10, style="Value.TLabel", relief="sunken")
        lab_t.grid(row=i+1, column=3, padx=2, pady=2, sticky="nsew")
        task_labels.append(lab_t)

    # 하단 조작부 (+ / - 버튼)
    f_ctrl = ttk.Frame(f_jog, padding=(0, 15))
    f_ctrl.grid(row=7, column=0, columnspan=4, sticky="nsew")

    jog_speed = tk.DoubleVar(value=20.0)
    ttk.Label(f_ctrl, text="Speed %:").pack(side=tk.LEFT)
    ttk.Spinbox(f_ctrl, from_=1, to=100, width=5, textvariable=jog_speed).pack(side=tk.LEFT, padx=5)

    def do_jog(direction):
        idx = selected_axis.get()
        speed = jog_speed.get() * direction
        node.execute_jog(idx, speed, ref=0)

    def stop_jog():
        node.move_stop_soft()

    # 큰 +/- 버튼 (이미지의 5번 영역)
    btn_minus = tk.Button(f_ctrl, text="—", font=("Arial", 20, "bold"), bg="#ff6b6b", fg="white",
                          command=lambda: do_jog(-1))
    btn_minus.pack(side=tk.LEFT, expand=True, fill=tk.BOTH, padx=5)
    
    btn_plus = tk.Button(f_ctrl, text="+", font=("Arial", 20, "bold"), bg="#1dd1a1", fg="white",
                         command=lambda: do_jog(1))
    btn_plus.pack(side=tk.LEFT, expand=True, fill=tk.BOTH, padx=5)

    ttk.Button(f_ctrl, text="STOP", command=stop_jog).pack(side=tk.LEFT, padx=5)

    # --- X,Y,Z축 정렬 (ParallelAxis2: 툴 축을 월드/베이스 좌표계 축과 평행하게) ---
    f_align = ttk.LabelFrame(f_jog, text="축 정렬 (X/Y/Z)", padding=5)
    f_align.grid(row=8, column=0, columnspan=4, sticky="ew", pady=(10, 0))

    align_ref_var = tk.IntVar(value=2)  # 0=Base, 2=World (기본: World)
    ttk.Label(f_align, text="좌표계:").pack(side=tk.LEFT, padx=(0, 5))
    ttk.Radiobutton(f_align, text="Base", variable=align_ref_var, value=0).pack(side=tk.LEFT, padx=2)
    ttk.Radiobutton(f_align, text="World", variable=align_ref_var, value=2).pack(side=tk.LEFT, padx=2)

    def on_align_axis(axis):
        def _run():
            def done(ok):
                root.after(
                    0,
                    lambda: messagebox.showinfo("축 정렬", "정렬 완료")
                    if ok
                    else messagebox.showerror("축 정렬", "축 정렬 실패 또는 서비스 없음"),
                )
            node.parallel_axis2_align(axis, ref=align_ref_var.get(), done_callback=done)
        threading.Thread(target=_run, daemon=True).start()

    ttk.Button(f_align, text="툴 X → 기준 X", command=lambda: on_align_axis(0)).pack(side=tk.LEFT, padx=5)
    ttk.Button(f_align, text="툴 Y → 기준 Y", command=lambda: on_align_axis(1)).pack(side=tk.LEFT, padx=5)
    ttk.Button(f_align, text="툴 Z → 기준 Z", command=lambda: on_align_axis(2)).pack(side=tk.LEFT, padx=5)

    # --- [Go to position] 리스트 입력 후 Go ---
    f_go = ttk.LabelFrame(root, text="Go to Position (리스트 입력)", padding=10)
    f_go.pack(fill=tk.X, padx=10, pady=5)

    go_type_var = tk.StringVar(value="posj")
    row_go = ttk.Frame(f_go)
    row_go.pack(fill=tk.X, pady=2)
    ttk.Radiobutton(row_go, text="PosX (x,y,z,rx,ry,rz mm,deg)", variable=go_type_var, value="posx").pack(side=tk.LEFT, padx=5)
    ttk.Radiobutton(row_go, text="PosJ (j1..j6 deg)", variable=go_type_var, value="posj").pack(side=tk.LEFT, padx=5)

    go_entry_var = tk.StringVar(value="0, 0, 90, 0, 90, 0")
    ttk.Label(f_go, text="값 (쉼표로 구분 6개):").pack(anchor=tk.W)
    go_entry = ttk.Entry(f_go, textvariable=go_entry_var, width=60)
    go_entry.pack(fill=tk.X, pady=2)

    def on_go():
        raw = go_entry_var.get().strip()
        try:
            vals = [float(x.strip()) for x in raw.split(",") if x.strip()]
        except ValueError:
            messagebox.showerror("Go", "숫자 6개를 쉼표로 구분해 입력하세요.")
            return
        if len(vals) < 6:
            messagebox.showerror("Go", "값이 6개 필요합니다 (현재 %d개)." % len(vals))
            return
        vals = vals[:6]
        go_t = go_type_var.get()

        def _run():
            def done(ok):
                root.after(0, lambda: (
                    messagebox.showinfo("Go", "이동 완료") if ok
                    else messagebox.showerror("Go", "이동 실패")
                ))
            if go_t == "posx":
                node.go_to_posx(vals, done_callback=done)
            else:
                node.go_to_posj(vals, done_callback=done)

        threading.Thread(target=_run, daemon=True).start()

    ttk.Button(f_go, text="Go", command=on_go).pack(pady=5)

    # --- [하단] Save & List ---
    f_bottom = ttk.LabelFrame(root, text="Position Management", padding=10)
    f_bottom.pack(fill=tk.BOTH, expand=True, padx=10, pady=5)

    # Gripper open % 설정
    gripper_var = tk.DoubleVar(value=50.0)
    row_g = ttk.Frame(f_bottom)
    row_g.pack(fill=tk.X, pady=2)
    ttk.Label(row_g, text="Gripper open % (100=open, 0=grip):").pack(side=tk.LEFT)
    spin_g = ttk.Spinbox(row_g, from_=0, to=100, width=5, textvariable=gripper_var)
    spin_g.pack(side=tk.LEFT, padx=5)

    def on_set_gripper():
        try:
            val = float(gripper_var.get())
        except (TypeError, ValueError):
            messagebox.showerror("Gripper", "Invalid value")
            return
        ok = node.set_gripper_open_percent(val)
        if not ok:
            messagebox.showerror("Gripper", "No gripper subscriber (dh_gripper_driver running?)")
        else:
            messagebox.showinfo("Gripper", f"Set gripper open to {val:.1f}%")

    ttk.Button(row_g, text="Set", command=on_set_gripper).pack(side=tk.LEFT, padx=5)

    save_as_var = tk.StringVar(value="both")
    row_s = ttk.Frame(f_bottom)
    row_s.pack(fill=tk.X)
    for v in ["posx", "posj", "both", "gripper"]:
        ttk.Radiobutton(row_s, text=v, variable=save_as_var, value=v).pack(side=tk.LEFT, padx=5)
    
    def on_save():
        def _task():
            save_as = save_as_var.get()
            # gripper 저장 시 현재 스핀박스 값을 노드에 반영 후 저장
            if save_as == "gripper":
                try:
                    node.set_gripper_open_percent(float(gripper_var.get()))
                except (TypeError, ValueError):
                    root.after(0, lambda: messagebox.showerror("Save", "Gripper 값이 올바르지 않습니다."))
                    return
            ok, px, pj = node.save_position(save_as=save_as)
            if ok:
                root.after(0, lambda: [refresh_list(), messagebox.showinfo("Save", "Success")])
        threading.Thread(target=_task, daemon=True).start()

    ttk.Button(row_s, text="Save Current", command=on_save).pack(side=tk.RIGHT, padx=5)

    listbox = tk.Listbox(f_bottom, height=5)
    listbox.pack(fill=tk.BOTH, expand=True, pady=5)

    def refresh_list():
        """저장 목록만 다시 읽어와 리스트박스에 표시 (Position 1, Gripper 1, ...)."""
        listbox.delete(0, tk.END)
        for label in node.get_saved_labels():
            listbox.insert(tk.END, label)

    def on_refresh_clear_all():
        """모든 저장 포지션 삭제 + 화면 값 리셋."""
        if messagebox.askyesno("Clear All", "모든 저장된 포지션을 삭제하시겠습니까?"):
            node.clear_saved()
            refresh_list()
            # Joint/Task 표시도 리셋
            for lab in joint_labels:
                lab.config(text="0.00")
            for lab in task_labels:
                lab.config(text="0.00")

    def on_delete_selected():
        """선택된 항목 하나만 삭제."""
        sel = listbox.curselection()
        if not sel:
            messagebox.showwarning("Delete", "삭제할 항목을 선택하세요.")
            return
        idx = sel[0]
        labels = node.get_saved_labels()
        name = labels[idx] if idx < len(labels) else f"항목 {idx}"
        if not messagebox.askyesno("Delete", f"{name} 를 삭제하시겠습니까?"):
            return
        if node.delete_position(idx):
            refresh_list()
        else:
            messagebox.showerror("Delete", "삭제에 실패했습니다.")

    btn_row = ttk.Frame(f_bottom)
    btn_row.pack(fill=tk.X)
    ttk.Button(btn_row, text="Refresh (Clear All)", command=on_refresh_clear_all).pack(side=tk.LEFT, expand=True, fill=tk.X)
    ttk.Button(btn_row, text="Delete Selected", command=on_delete_selected).pack(side=tk.LEFT, expand=True, fill=tk.X)
    def on_load_selected():
        sel = listbox.curselection()
        if not sel:
            messagebox.showwarning("Load", "불러올 항목을 선택하세요.")
            return
        info = node.load_position(sel[0])
        if info is None:
            messagebox.showerror("Load", "항목을 불러올 수 없습니다.")
            return
        kind = info.get("kind", "?")
        lines = [f"타입: {kind}"]
        if info.get("posx") is not None:
            lines.append(f"posx: {info['posx']}")
        if info.get("posj") is not None:
            lines.append(f"posj: {info['posj']}")
        if info.get("gripper") is not None:
            lines.append(f"gripper: {info['gripper']:.1f}%")
        messagebox.showinfo("Load Selected", "\n".join(lines))

    ttk.Button(btn_row, text="Load Selected", command=on_load_selected).pack(side=tk.LEFT, expand=True, fill=tk.X)

    # --- Do Scenario (pick_and_place 연동) ---
    def on_do_scenario():
        """pick_and_place 노드가 scenario_positions.json 기반 시나리오를 실행하도록 트리거."""
        def _task():
            try:
                node.trigger_do_pick_and_place("go")
                root.after(0, lambda: messagebox.showinfo("Do Scenario", "Sent /do_pick_and_place (go)."))
            except Exception as e:
                root.after(0, lambda: messagebox.showerror("Do Scenario", f"Failed: {e}"))
        threading.Thread(target=_task, daemon=True).start()

    ttk.Button(f_bottom, text="Do Scenario (do_pick_and_place)", command=on_do_scenario).pack(
        fill=tk.X, padx=5, pady=5
    )

    # --- [루프] 데이터 업데이트 ---
    def update_loop():
        # 좌표 및 Solution Space 업데이트
        def _get_done(posx, posj, solution_space, err):
            if not err:
                for i in range(6):
                    joint_labels[i].config(text=_format_pos(posj[i]))
                    task_labels[i].config(text=_format_pos(posx[i]))
            if solution_space is not None:
                sol_space_label.config(text=str(solution_space))
            else:
                sol_space_label.config(text="—")
        node.get_current_position(_get_done)
        
        # 선택된 축 버튼 강조 (간단히 색상 변경)
        idx = selected_axis.get()
        # 모든 버튼 초기화 후 현재 선택된 버튼만 색상 변경하는 로직 추가 가능
        
        node.spin_once_and_drain_futures()
        root.after(2000, update_loop)

    refresh_list()
    update_loop()
    root.mainloop()
