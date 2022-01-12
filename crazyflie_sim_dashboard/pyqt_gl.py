import numpy as np
from pyqtgraph.Qt import QtCore, QtGui
import pyqtgraph.opengl as gl
import sys
from opensimplex import OpenSimplex

class Terrain(object):
    def __init__(self):
        self.app = QtGui.QApplication(sys.argv)
        self.w = gl.GLViewWidget()
        self.w.setGeometry(0,110,1920,1080)
        self.w.show()
        self.w.setWindowTitle('Terrain')
        self.w.setCameraPosition(distance=30,elevation=8)

        ## ADD GRID
        grid = gl.GLGridItem()
        grid.scale(2,2,2)
        self.w.addItem(grid)

        self.nsteps = 1 # Distance between each vertice
        self.xpoints = range(-20,22,self.nsteps)
        self.ypoints = range(-20,22,self.nsteps)
        self.nfaces = len(self.ypoints)
        self.tmp = OpenSimplex()

        ## GENERATE VERTICES
        verts = np.array([
            [x, y, 1.5*self.tmp.noise2(x=n/5 ,y=m/5)] for n,x in enumerate(self.xpoints) for m,y in enumerate(self.ypoints)
        ], dtype=np.float32)

        # for i in verts:
        #     print(i)


        ## GENERATE FACES
        faces = []
        colors = []

        for m in range(self.nfaces - 1):
            y_offset = m*self.nfaces
            for n in range(self.nfaces - 1):
                faces.append([n+y_offset, y_offset+n+self.nfaces, y_offset+n+self.nfaces+1])
                colors.append([0,0,0,0])

                faces.append([n+y_offset, y_offset+n+1, y_offset+n+self.nfaces+1])
                colors.append([0,0,0,0])

        faces = np.array(faces)
        colors = np.array(colors)

        ## GENERATE MESH SURFACE
        self.m1 = gl.GLMeshItem(
            vertexes=verts,
            faces=faces, faceColors = colors,
            smooth=False, drawEdges=True,
        )
        self.m1.setGLOptions('additive')
        self.w.addItem(self.m1)



    def start(self):
        if (sys.flags.interactive != 1) or not hasattr(QtCore, 'PYQQT_VERSION'):
            QtGui.QApplication.instance().exec_()

    



if __name__ == '__main__':
    t = Terrain()
    t.start()