#!/usr/bin/env python3
"""
CSV标签修改工具 - GUI版本

将压腿(2)状态改为被动状态(4)，用于验证压腿时使用被动状态参数的效果。

修改规则：
- 将所有 m1_state_label=2 改为 m1_state_label=4
- 将所有 m2_state_label=2 改为 m2_state_label=4
"""

import sys
import csv
import os
import tkinter as tk
from tkinter import ttk, filedialog, messagebox, scrolledtext
from threading import Thread


class CSVLabelModifier:
    """CSV标签修改器"""

    def __init__(self, log_callback=None):
        self.log_callback = log_callback

    def log(self, msg):
        if self.log_callback:
            self.log_callback(msg)
        else:
            print(msg)

    def modify_labels(self, input_file, output_file):
        """
        修改CSV标签：将所有2改为4

        Args:
            input_file: 输入CSV文件
            output_file: 输出CSV文件
        """
        self.log(f"读取文件: {input_file}")

        with open(input_file, 'r', encoding='utf-8') as f:
            reader = csv.reader(f)
            rows = list(reader)

        if len(rows) < 2:
            self.log("错误: CSV文件数据不足")
            return False

        header = rows[0]
        data = rows[1:]

        # 查找列索引
        try:
            m1_label_idx = header.index('m1_state_label')
            m2_label_idx = header.index('m2_state_label')
        except ValueError as e:
            self.log(f"错误: 找不到必要的列 - {e}")
            return False

        self.log(f"加载 {len(data)} 条数据")
        self.log(f"修改规则: 2(压腿) -> 4(被动)")

        m1_modified_count = 0
        m2_modified_count = 0

        for row in data:
            try:
                m1_label = int(row[m1_label_idx])
                m2_label = int(row[m2_label_idx])
            except (ValueError, IndexError):
                continue

            # 将2改为4
            if m1_label == 2:
                row[m1_label_idx] = '4'
                m1_modified_count += 1

            if m2_label == 2:
                row[m2_label_idx] = '4'
                m2_modified_count += 1

        # 写入输出文件
        with open(output_file, 'w', encoding='utf-8', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(header)
            writer.writerows(data)

        self.log("")
        self.log("=" * 40)
        self.log("修改完成!")
        self.log(f"  电机1: {m1_modified_count} 条标签修改 (2->4)")
        self.log(f"  电机2: {m2_modified_count} 条标签修改 (2->4)")
        self.log(f"  输出文件: {output_file}")
        self.log("=" * 40)

        return True


class ModifierGUI:
    """CSV标签修改器GUI界面"""

    def __init__(self):
        self.root = tk.Tk()
        self.root.title("CSV标签修改工具 - 压腿改被动状态")
        self.root.geometry("700x500")
        self.root.resizable(True, True)

        self.input_file = tk.StringVar()
        self.output_file = tk.StringVar()

        self.modifier = CSVLabelModifier(log_callback=self.log)

        self._create_widgets()

    def _create_widgets(self):
        # 主框架
        main_frame = ttk.Frame(self.root, padding="10")
        main_frame.pack(fill=tk.BOTH, expand=True)

        # 说明文字
        desc_frame = ttk.LabelFrame(main_frame, text="功能说明", padding="5")
        desc_frame.pack(fill=tk.X, pady=(0, 10))

        desc_text = """将所有压腿(2)状态改为被动状态(4)，用于验证压腿时使用被动状态参数的效果。

状态标签说明:
  0 = 空闲(idle)      1 = 抬腿(phase1)    2 = 压腿(phase2)
  3 = 等待(waiting)   4 = 被动(passive)

修改规则: 2 -> 4"""

        ttk.Label(desc_frame, text=desc_text, justify=tk.LEFT).pack(anchor=tk.W)

        # 文件选择框架
        file_frame = ttk.LabelFrame(main_frame, text="文件设置", padding="10")
        file_frame.pack(fill=tk.X, pady=(0, 10))

        # 输入文件
        ttk.Label(file_frame, text="输入文件:").grid(row=0, column=0, sticky=tk.W, pady=5)
        ttk.Entry(file_frame, textvariable=self.input_file, width=50).grid(row=0, column=1, padx=5, pady=5)
        ttk.Button(file_frame, text="浏览...", command=self._browse_input).grid(row=0, column=2, pady=5)

        # 输出文件
        ttk.Label(file_frame, text="输出文件:").grid(row=1, column=0, sticky=tk.W, pady=5)
        ttk.Entry(file_frame, textvariable=self.output_file, width=50).grid(row=1, column=1, padx=5, pady=5)
        ttk.Button(file_frame, text="浏览...", command=self._browse_output).grid(row=1, column=2, pady=5)

        # 按钮框架
        btn_frame = ttk.Frame(main_frame)
        btn_frame.pack(fill=tk.X, pady=(0, 10))

        self.process_btn = ttk.Button(btn_frame, text="开始处理", command=self._start_process)
        self.process_btn.pack(side=tk.LEFT, padx=5)

        ttk.Button(btn_frame, text="清空日志", command=self._clear_log).pack(side=tk.LEFT, padx=5)

        # 日志框架
        log_frame = ttk.LabelFrame(main_frame, text="处理日志", padding="5")
        log_frame.pack(fill=tk.BOTH, expand=True)

        self.log_text = scrolledtext.ScrolledText(log_frame, height=15, font=("Consolas", 9))
        self.log_text.pack(fill=tk.BOTH, expand=True)

    def _browse_input(self):
        filename = filedialog.askopenfilename(
            title="选择输入CSV文件",
            filetypes=[("CSV文件", "*.csv"), ("所有文件", "*.*")]
        )
        if filename:
            self.input_file.set(filename)
            # 自动生成输出文件名
            base, ext = os.path.splitext(filename)
            self.output_file.set(f"{base}_passive{ext}")

    def _browse_output(self):
        filename = filedialog.asksaveasfilename(
            title="选择输出CSV文件",
            filetypes=[("CSV文件", "*.csv"), ("所有文件", "*.*")],
            defaultextension=".csv"
        )
        if filename:
            self.output_file.set(filename)

    def log(self, msg):
        self.log_text.insert(tk.END, msg + "\n")
        self.log_text.see(tk.END)
        self.root.update_idletasks()

    def _clear_log(self):
        self.log_text.delete(1.0, tk.END)

    def _start_process(self):
        input_file = self.input_file.get().strip()
        output_file = self.output_file.get().strip()

        if not input_file:
            messagebox.showerror("错误", "请选择输入文件")
            return

        if not os.path.exists(input_file):
            messagebox.showerror("错误", f"输入文件不存在: {input_file}")
            return

        if not output_file:
            messagebox.showerror("错误", "请指定输出文件")
            return

        self.process_btn.config(state=tk.DISABLED)
        self._clear_log()

        def process():
            try:
                success = self.modifier.modify_labels(input_file, output_file)
                if success:
                    self.root.after(0, lambda: messagebox.showinfo("完成", "CSV标签修改完成!"))
                else:
                    self.root.after(0, lambda: messagebox.showerror("失败", "处理失败，请查看日志"))
            except Exception as e:
                self.log(f"错误: {e}")
                self.root.after(0, lambda: messagebox.showerror("错误", str(e)))
            finally:
                self.root.after(0, lambda: self.process_btn.config(state=tk.NORMAL))

        Thread(target=process, daemon=True).start()

    def run(self):
        self.root.mainloop()


def main():
    # 如果有命令行参数，使用命令行模式
    if len(sys.argv) >= 2:
        input_file = sys.argv[1]

        if len(sys.argv) >= 3:
            output_file = sys.argv[2]
        else:
            base, ext = os.path.splitext(input_file)
            output_file = f"{base}_passive{ext}"

        print(f"输入文件: {input_file}")
        print(f"输出文件: {output_file}")
        print(f"修改规则: 2(压腿) -> 4(被动)")
        print()

        if not os.path.exists(input_file):
            print(f"文件不存在: {input_file}")
            sys.exit(1)

        modifier = CSVLabelModifier()
        if modifier.modify_labels(input_file, output_file):
            print("\n处理成功!")
        else:
            print("\n处理失败!")
            sys.exit(1)
    else:
        # GUI模式
        app = ModifierGUI()
        app.run()


if __name__ == "__main__":
    main()
