# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'waypoint.ui'
#
# Created by: PyQt5 UI code generator 5.9
#
# WARNING! All changes made in this file will be lost!

from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtCore import Qt, QPoint, pyqtSignal
from PyQt5.QtGui import QPixmap, QPainter, QPen
from PyQt5.QtWidgets import QWidget, QLabel

from PyQt5 import QtCore, QtWidgets, QtWebEngineWidgets
from folium.plugins import Draw
import folium, io, sys, json
from pyroutelib3 import Router # Import the router

class myLabel(QLabel):
    clicked = pyqtSignal()



class Ui_Dialog(object):
        router = Router("foot") # Initialise it
    m = folium.Map(location=[10.77379, 106.65995], zoom_start=17)
    check = False
    draw = Draw(
        draw_options={
            'polyline':False,
            'rectangle':True,
            'polygon':True,
            'circle':False,
            'marker':True,
            'circlemarker':False},
        edit_options={'edit':False})
    m.add_child(draw)

    waypoints = []
    data = io.BytesIO()
    m.save(data, close_file=False)

    class WebEnginePage(QtWebEngineWidgets.QWebEnginePage):
       global waypoints,m,data,view
       def javaScriptConsoleMessage(self, level, msg, line, sourceID):
        coords_dict = json.loads(msg)
        coords = coords_dict['geometry']['coordinates']
        print(coords[0])
        print(coords[1])
        waypoints.append(router.findNode(coords[1],coords[0]))
        
        if(len(waypoints) == 2):
            status, route = router.doRoute(waypoints[0], waypoints[1]) # Find the route - a list of OSM nodes

            if status == 'success':
                routeLatLons = list(map(router.nodeLatLon, route)) # Get actual route coordinates
                for coord in routeLatLons:
                    folium.CircleMarker( location=[ coord[0], coord[1] ], fill_color='#43d9de', radius=1 ).add_to( m )
                    print(coord)
                print(routeLatLons)
            # 
            m
            data = io.BytesIO()
            m.save(data, close_file=False)
            view.setPage(page)
            view.setHtml(data.getvalue().decode())
            waypoints.clear()

    view = QtWebEngineWidgets.QWebEngineView()
    page = WebEnginePage(view)
    view.setPage(page)
    def setupUi(self, Dialog):
        Dialog.setObjectName("Dialog")
        Dialog.resize(857, 892)
        self.buttonBox = QtWidgets.QDialogButtonBox(Dialog)
        self.buttonBox.setGeometry(QtCore.QRect(510, 850, 341, 32))
        self.buttonBox.setOrientation(QtCore.Qt.Horizontal)
        self.buttonBox.setStandardButtons(QtWidgets.QDialogButtonBox.Cancel|QtWidgets.QDialogButtonBox.Ok)
        self.buttonBox.setObjectName("buttonBox")
        self.groupBox = QtWidgets.QGroupBox(Dialog)
        self.groupBox.setGeometry(QtCore.QRect(10, 10, 841, 841))
        self.groupBox.setObjectName("groupBox")

        self.label = myLabel(self.groupBox)
        self.label.setGeometry(QtCore.QRect(10, 20, 831, 821))
        self.label.setObjectName("label")
        
        self.image = QPixmap("map_png.PNG")
        # self.setGeometry(100, 100, 500, 300)
        self.label.resize(self.image.width(), self.image.height())

        self.retranslateUi(Dialog)
        self.buttonBox.accepted.connect(Dialog.accept)
        self.buttonBox.rejected.connect(Dialog.reject)
        QtCore.QMetaObject.connectSlotsByName(Dialog)
    
    def retranslateUi(self, Dialog):
        _translate = QtCore.QCoreApplication.translate
        Dialog.setWindowTitle(_translate("Dialog", "Dialog"))
        self.groupBox.setTitle(_translate("Dialog", "Map"))
        self.label.setPixmap(self.image)
       

if __name__ == "__main__":
    import sys
    app = QtWidgets.QApplication(sys.argv)


    Dialog = QtWidgets.QDialog()
    ui = Ui_Dialog()
    ui.setupUi(Dialog)
    Dialog.show()
    sys.exit(app.exec_())

