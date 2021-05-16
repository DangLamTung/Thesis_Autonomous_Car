import sys
import io
import folium # pip install folium
from PyQt5.QtWidgets import QApplication, QWidget, QHBoxLayout, QVBoxLayout
from PyQt5.QtWebEngineWidgets import QWebEngineView # pip install PyQtWebEngine

"""
Folium in PyQt5
"""
class MyApp(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle('Folium in PyQt Example')
        self.window_width, self.window_height = 640,480
        self.setMinimumSize(self.window_width, self.window_height)

        layout = QVBoxLayout()
        self.setLayout(layout)

        coordinate = (10.77379, 106.65995)
        m = folium.Map(
        	tiles='OpenStreetMap',
        	zoom_start=13,
        	location=coordinate
        )

        folium.ClickForMarker().add_to(m)
        folium.LatLngPopup().add_to(m)

        # m.on_interaction(handle_map_interaction)
        # save map data to data object
        data = io.BytesIO()
        m.save(data, close_file=False)

        webView = QWebEngineView()
        webView.setHtml(data.getvalue().decode())
        layout.addWidget(webView)

    def handle_map_interaction(**kwargs):
        print(kwargs)
if __name__ == '__main__':
    app = QApplication(sys.argv)
    app.setStyleSheet('''
        QWidget {
            font-size: 35px;
        }
    ''')
    
    myApp = MyApp()
    myApp.show()

    try:
        sys.exit(app.exec_())
    except SystemExit:
        print('Closing Window...')