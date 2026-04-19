import tkinter as tk
from tkinter import ttk
import pygame

# ----------------------------
# Init pygame joystick
# ----------------------------
pygame.init()
pygame.joystick.init()

# ----------------------------
# GUI App
# ----------------------------
class JoystickInspector:
    def __init__(self, root):
        self.root = root
        self.root.title("Joystick Inspector")
        self.root.geometry("700x500")

        self.selected_joystick = None
        self.axis_labels = {}
        self.button_labels = {}

        self.build_ui()
        self.detect_joysticks()
        self.update_loop()

    # ----------------------------
    # UI Layout
    # ----------------------------
    def build_ui(self):
        top_frame = ttk.Frame(self.root)
        top_frame.pack(fill="x", pady=10)

        ttk.Label(top_frame, text="Available Joysticks:").pack(side="left")
        self.joy_combo = ttk.Combobox(top_frame, state="readonly", width=40)
        self.joy_combo.pack(side="left", padx=10)
        self.joy_combo.bind("<<ComboboxSelected>>", self.select_joystick)

        self.info_label = ttk.Label(self.root, text="")
        self.info_label.pack(pady=5)

        # Axis frame
        self.axis_frame = ttk.LabelFrame(self.root, text="Axes")
        self.axis_frame.pack(fill="both", expand=True, padx=10, pady=5)

        # Button frame
        self.button_frame = ttk.LabelFrame(self.root, text="Buttons")
        self.button_frame.pack(fill="both", expand=True, padx=10, pady=5)

    # ----------------------------
    # Detect Joysticks
    # ----------------------------
    def detect_joysticks(self):
        count = pygame.joystick.get_count()
        names = []

        for i in range(count):
            joy = pygame.joystick.Joystick(i)
            joy.init()
            names.append(f"[{i}] {joy.get_name()}")

        self.joy_combo["values"] = names

        if count > 0:
            self.joy_combo.current(0)
            self.select_joystick()

        self.info_label.config(text=f"Detected {count} joystick(s)")

    # ----------------------------
    # Select Joystick
    # ----------------------------
    def select_joystick(self, event=None):
        idx = self.joy_combo.current()
        self.selected_joystick = pygame.joystick.Joystick(idx)
        self.selected_joystick.init()

        self.build_axis_labels()
        self.build_button_labels()

    # ----------------------------
    # Build Axis UI
    # ----------------------------
    def build_axis_labels(self):
        for widget in self.axis_frame.winfo_children():
            widget.destroy()

        self.axis_labels.clear()
        n_axes = self.selected_joystick.get_numaxes()

        for i in range(n_axes):
            lbl = ttk.Label(self.axis_frame, text=f"Axis {i}: 0.000", font=("Consolas", 11))
            lbl.grid(row=i // 2, column=i % 2, sticky="w", padx=10)
            self.axis_labels[i] = lbl

    # ----------------------------
    # Build Button UI
    # ----------------------------
    def build_button_labels(self):
        for widget in self.button_frame.winfo_children():
            widget.destroy()

        self.button_labels.clear()
        n_buttons = self.selected_joystick.get_numbuttons()

        for i in range(n_buttons):
            lbl = ttk.Label(self.button_frame, text=f"Button {i}: OFF", font=("Consolas", 11))
            lbl.grid(row=i // 4, column=i % 4, sticky="w", padx=10)
            self.button_labels[i] = lbl

    # ----------------------------
    # Update Loop
    # ----------------------------
    def update_loop(self):
        pygame.event.pump()

        if self.selected_joystick:
            # Axes
            for i, lbl in self.axis_labels.items():
                val = self.selected_joystick.get_axis(i)
                lbl.config(text=f"Axis {i}: {val:+.3f}")

            # Buttons
            for i, lbl in self.button_labels.items():
                val = self.selected_joystick.get_button(i)
                state = "ON" if val else "OFF"
                lbl.config(text=f"Button {i}: {state}")

        self.root.after(50, self.update_loop)


# ----------------------------
# Run App
# ----------------------------
if __name__ == "__main__":
    root = tk.Tk()
    app = JoystickInspector(root)
    root.mainloop()
