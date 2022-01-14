## build a QApplication before building other widgets
import pyqtgraph as pg
pg.mkQApp()

## make a widget for displaying 3D objects
import pyqtgraph.opengl as gl
view = gl.GLViewWidget()
view.show()

## create three grids, add each to the view
xgrid = gl.GLGridItem()
ygrid = gl.GLGridItem()
zgrid = gl.GLGridItem()
view.addItem(xgrid)
view.addItem(ygrid)
view.addItem(zgrid)

## rotate x and y grids to face the correct direction
xgrid.rotate(90, 0, 1, 0)
ygrid.rotate(90, 1, 0, 0)

## scale each grid differently
xgrid.scale(0.2, 0.1, 0.1)
ygrid.scale(0.2, 0.1, 0.1)
zgrid.scale(0.1, 0.2, 0.1)