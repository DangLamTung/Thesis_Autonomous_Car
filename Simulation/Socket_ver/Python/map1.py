from PyQt5 import QtCore, QtWidgets, QtWebEngineWidgets
from folium.plugins import Draw
import folium, io, sys, json
from pyroutelib3 import Router # Import the router

if __name__ == '__main__': 
    app = QtWidgets.QApplication(sys.argv)
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
view.setHtml(data.getvalue().decode())
view.show()
sys.exit(app.exec_())