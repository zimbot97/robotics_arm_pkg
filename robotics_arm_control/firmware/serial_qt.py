import sys
import serial
from PyQt5.QtWidgets import (
    QApplication, QWidget, QVBoxLayout, QHBoxLayout,
    QSlider, QLabel
)
from PyQt5.QtCore import Qt, QTimer

class SerialSlider(QWidget):
    def __init__(self, port="/dev/ttyUSB0", baud=115200):
        super().__init__()

        # Setup serial
        try:
            self.ser = serial.Serial(port, baud, timeout=1)
        except Exception as e:
            print(f"⚠️ Could not open serial port {port}: {e}")
            self.ser = None

        # GUI setup
        self.setWindowTitle("Six-Servo Controller")
        self.resize(500, 300)
        layout = QVBoxLayout()

        self.sliders = []
        self.value_labels = []

        for i in range(6):
            row = QHBoxLayout()
            MAX = 0
            MIN = 0
            SET = 0
            if(i == 5):
                MAX = 180
                MIN = 100
                SET = 180
            else:
                MAX = 180
                MIN = 0
                SET = 90
            
            # Slider
            slider = QSlider(Qt.Horizontal)
            slider.setMinimum(MIN)
            slider.setMaximum(MAX)
            slider.setValue(SET)
            slider.valueChanged.connect(self.on_value_changed)
            self.sliders.append(slider)
            row.addWidget(slider)

            # Sent value label
            val_label = QLabel(f"Servo {i}: {SET}")
            self.value_labels.append(val_label)
            row.addWidget(val_label)

            layout.addLayout(row)

        self.setLayout(layout)

        # Timer to poll serial feedback
        self.timer = QTimer()
        self.timer.timeout.connect(self.read_serial)
        self.timer.start(500)  # every 100 ms

    def on_value_changed(self, value):
        # Update labels
        for i, slider in enumerate(self.sliders):
            self.value_labels[i].setText(f"Servo {i}: {slider.value()}")

        # Send array of 6 values
        if self.ser and self.ser.is_open:
            values = [str(slider.value()) for slider in self.sliders]
            msg = ",".join(values) + "\n"
            self.ser.write(msg.encode("utf-8"))
            print(f"Sent: {msg.strip()}")

    def read_serial(self):
        if self.ser and self.ser.in_waiting:
            try:
                line = self.ser.readline().decode().strip()
                if line:
                    print(f"Received: {line}")
            except Exception as e:
                print(f"⚠️ Serial read error: {e}")

    def closeEvent(self, event):
        if self.ser and self.ser.is_open:
            self.ser.close()
        event.accept()

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = SerialSlider(port="/dev/ttyUSB0", baud=115200)  # adjust port/baud
    window.show()
    sys.exit(app.exec_())
