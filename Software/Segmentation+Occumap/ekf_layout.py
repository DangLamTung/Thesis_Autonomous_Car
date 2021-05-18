
from PyQt5.QtWidgets import QApplication,QTabWidget, QWidget, QPushButton, QHBoxLayout,QVBoxLayout,QSlider,QLabel,QMessageBox,QListWidget,QTextEdit
import pyqtgraph as pg
from list_port import list_serial_ports

class EKF_layout():
     def __init__(self):

        self.slider = QSlider()
        self.slider.setValue(0)
        self.slider.setMinimum(-180)
        self.slider.setMaximum(180)


        self.slider1 = QSlider()
        self.slider1.setValue(0)
        self.slider1.setMinimum(-500)
        self.slider1.setMaximum(500)
        # slider1.valueChanged.connect(update_slider)

        self.slider2 = QSlider()
        self.slider2.setValue(1000)
        self.slider2.setMinimum(1000)
        self.slider2.setMaximum(1500)
        # slider2.valueChanged.connect(update_slider)

        self.slider3 = QSlider()
        self.slider3.setValue(1000)
        self.slider3.setMinimum(1000)
        self.slider3.setMaximum(1500)
        # slider3.valueChanged.connect(update_slider)

        self.slider4 = QSlider()
        self.slider4.setValue(1000)
        self.slider4.setMinimum(1000)
        self.slider4.setMaximum(1500)
        # slider4.valueChanged.connect(setPID_all)

        self.slider5 = QSlider()
        self.slider5.setValue(0)
        self.slider5.setMaximum(1500)
        # slider5.valueChanged.connect(update_slider)

        self.slider6 = QSlider()
        self.slider6.setValue(0)
        self.slider6.setMaximum(1500)
        # slider6.valueChanged.connect(update_slider)

        self.slider7 = QSlider()
        self.slider7.setValue(0)
        self.slider7.setMaximum(9909)
        # 
        # Will print '5'

        self.layout = QHBoxLayout()
        self.layout_inner = QHBoxLayout()
        self.layout_inner1 = QHBoxLayout()
        self.layout_inner2 = QVBoxLayout()

        self.layout_value = QHBoxLayout()

        self.label = QLabel("Angle")
        self.label1 = QLabel("PWM")
        self.label2 = QLabel("Motor 3")

        self.label3 = QLabel("Motor 4")
        self.label4 = QLabel("All motor")
        self.label5 = QLabel("Kd 2")

        self.label6 = QLabel("Motor 1")
        self.label7 = QLabel("Motor 2")

        self.label_duty1 = QLabel("Gyro x")
        self.label_duty2 = QLabel("Gyro y")
        self.label_duty3 = QLabel("Gyro z")

        self.label_duty4 = QLabel("Acc x")
        self.label_duty5 = QLabel("Acc y")
        self.label_duty6 = QLabel("Acc z")

        self.label_duty7 = QLabel("Roll")
        self.label_duty8 = QLabel("Pitch")
        self.label_duty9 = QLabel("Yaw")

        self.label_duty10 = QLabel("Time: ")

        self.connect_button = QPushButton('Connect COM')
        

        self.detatch_button = QPushButton('Disconnect COM')
  
        self.layout_inner = QHBoxLayout()


        self.layout_inner2.addWidget(self.connect_button)
        self.layout_inner2.addWidget(self.detatch_button)
         
        self.serial_list = list_serial_ports()
        self.list_widget = QListWidget()
        for (i,ser_name) in enumerate(self.serial_list):
            self.list_widget.insertItem(i, ser_name)


        # 
        self.layout_inner.addWidget(self.list_widget)
        self.layout_inner.addLayout(self.layout_inner2)


        self.button = QPushButton('Emergency STOP')

        self.button3 = QPushButton('Arm')
        self.button4 = QPushButton('Test')
        self.button1 = QPushButton('Start')
    

        self.button2 = QPushButton('Stop')
      
        self.textEdit = QTextEdit()
        
        self.layout_inner1.addWidget(self.button)
        self.layout_inner1.addWidget(self.button3)
        self.layout_inner1.addWidget(self.button4)
        self.layout_inner1.addWidget(self.button1)
        self.layout_inner1.addWidget(self.button2)

        self.layout.addWidget(self.slider)
        self.layout.addWidget(self.label)

        self.layout.addWidget(self.slider1)
        self.layout.addWidget(self.label1)

        self.layout.addWidget(self.slider2)
        self.layout.addWidget(self.label2)

        self.layout.addWidget(self.slider3)
        self.layout.addWidget(self.label3)

        self.layout.addWidget(self.slider4)
        self.layout.addWidget(self.label4)

        self.layout.addWidget(self.slider5)
        self.layout.addWidget(self.label5)

        self.layout.addWidget(self.slider6)
        self.layout.addWidget(self.label6)
        
        self.layout_value.addWidget(self.label_duty1)
        self.layout_value.addWidget(self.label_duty2)
        self.layout_value.addWidget(self.label_duty3)
        self.layout_value.addWidget(self.label_duty10)


        self.layout.addStretch(2)
        
        self.layout.addLayout(self.layout_inner)
        
        self.plot_uart = pg.GraphicsLayoutWidget() # creates a window
        self.p = self.plot_uart.addPlot(title="Gyro X")  # creates empty space for the plot in the window

        self.plot_uart1 = pg.GraphicsLayoutWidget() # creates a window
        self.p1 = self.plot_uart1.addPlot(title="Gyro Y")  # creates empty space for the plot in the window

        self.plot_uart2 = pg.GraphicsLayoutWidget() # creates a window
        self.p2 = self.plot_uart2.addPlot(title="Gyro Z")  # creates empty space for the plot in the window

        self.plot_uart3 = pg.GraphicsLayoutWidget() # creates a window
        self.p3 = self.plot_uart3.addPlot(title="Roll")  # creates empty space for the plot in the window

        self.plot_uart4 = pg.GraphicsLayoutWidget() # creates a window
        self.p4 = self.plot_uart4.addPlot(title="Pitch")  # creates empty space for the plot in the window

        self.plot_uart5 = pg.GraphicsLayoutWidget() # creates a window
        self.p5 = self.plot_uart5.addPlot(title="Yaw")  # creates empty space for the plot in the window

        self.plot_uart6 = pg.GraphicsLayoutWidget() # creates a window
        self.p6 = self.plot_uart6.addPlot(title="Gyro X")  # creates empty space for the plot in the window

        self.plot_uart7 = pg.GraphicsLayoutWidget() # creates a window
        self.p7 = self.plot_uart7.addPlot(title="Gyro X")  # creates empty space for the plot in the window

        self.plot_uart8 = pg.GraphicsLayoutWidget() # creates a window
        self.p8 = self.plot_uart8.addPlot(title="Gyro X")  # creates empty space for the plot in the window

    
        self.curve = self.p.plot()                        # create an empty "plot" (a curve to plot)
        self.curve1 = self.p1.plot() 
        self.curve2 = self.p2.plot() 
        self.curve3 = self.p3.plot()                        # create an empty "plot" (a curve to plot)
        self.curve4 = self.p4.plot() 
        self.curve5 = self.p5.plot() 

        self.curve6 = self.p6.plot()                        # create an empty "plot" (a curve to plot)
        self.curve7 = self.p7.plot() 
        self.curve8 = self.p8.plot() 
        self.plot_batch = QVBoxLayout()
        self.layout_plot = QHBoxLayout()
        self.layout_plot.addWidget(self.plot_uart)
        self.layout_plot.addWidget(self.plot_uart1)
        self.layout_plot.addWidget(self.plot_uart2)

        self.layout_plot1 = QHBoxLayout()
        self.layout_plot1.addWidget(self.plot_uart3)
        self.layout_plot1.addWidget(self.plot_uart4)
        self.layout_plot1.addWidget(self.plot_uart5)

        self.layout_plot2 = QHBoxLayout()
        self.layout_plot2.addWidget(self.plot_uart6)
        self.layout_plot2.addWidget(self.plot_uart7)
        self.layout_plot2.addWidget(self.plot_uart8)

        self.plot_batch.addLayout(self.layout_plot)
        self.plot_batch.addLayout(self.layout_plot1)
        self.plot_batch.addLayout(self.layout_plot2)
