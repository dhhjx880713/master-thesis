import numpy as np
import matplotlib.pyplot as plt
import mpl_toolkits.mplot3d.axes3d as p3
import matplotlib.animation as animation

class visualizer():
    
    def __init__(self, hips, worldpos,skeleton, filename = None):
        self.skeleton = skeleton
        self.hips = hips
        #self.getSkeleton(hips)
        
        self.worldpos = worldpos
        self.filename = filename
        self.fig = plt.figure()
        self.ax = p3.Axes3D(self.fig)
        self.frameLen = len(worldpos)
        self.origin = worldpos[0][hips.name]
        self.boundary()

        
        
##    def getSkeleton(self, joint ):
##        for child in joint.children:
##            self.skeleton.append([joint.name, child.name])
##        for child in joint.children:
##            self.getSkeleton(child)
    def boundary(self):
        self.xmin = self.origin[0]
        self.xmax = self.origin[0]
        self.ymin = self.origin[1]
        self.ymax = self.origin[1]
        self.zmin = self.origin[2]
        self.zmax = self.origin[2]
        for frame in self.worldpos:
            for joint in self.worldpos[frame]:
                self.xmin = min(self.xmin, self.worldpos[frame][joint][0])
                self.xmax = max(self.xmax, self.worldpos[frame][joint][0])
                self.ymin = min(self.ymin, self.worldpos[frame][joint][1])
                self.ymax = max(self.ymax, self.worldpos[frame][joint][1])
                self.zmin = min(self.zmin, self.worldpos[frame][joint][2])
                self.zmax = max(self.zmax, self.worldpos[frame][joint][2])
        offset = 4000
        xInterval = self.xmax - self.xmin
        yInterval = self.ymax - self.ymin
        zInterval = self.zmax - self.zmin
        factor = max(xInterval, yInterval)/zInterval
        maxInterval = max(xInterval, yInterval, zInterval)

        if maxInterval < offset:
            self.xmin = self.xmin - (offset-xInterval)/2.0
            self.xmax = self.xmax + (offset-xInterval)/2.0
            self.ymin = self.ymin - (offset-yInterval)/2.0
            self.ymax = self.ymax + (offset-yInterval)/2.0
            self.zmax = self.zmax + (offset-zInterval)/2.0
        else:
            self.zmax = self.zmax + zInterval*factor/4.0
##        print self.xmin
##        print self.xmax
##        print self.ymin
##        print self.ymax
##        print self.zmin
##        print self.zmax

            
    def plotOneFrame(self, frame_num):
        fig = plt.figure(1)
        ax = p3.Axes3D(fig)
        for item in self.skeleton:
            tmp = [self.worldpos[frame_num][item[0]], self.worldpos[frame_num][item[1]]]
            tmp = np.transpose(np.array(tmp))
            ax.plot(*tmp, color = 'b')
##        self.ax.set_xlim3d(-2000+self.origin[0], 2000+self.origin[0])
##        self.ax.set_ylim3d(-2000+self.origin[1], 2000+self.origin[1])
##        self.ax.set_zlim3d(-1000+self.origin[2], 2000+self.origin[2])
        #ax.set_aspect('equal','box')
##        ax.set_xlim3d(-2000+self.origin[0], 2000+self.origin[0])
##        ax.set_ylim3d(-2000+self.origin[1], 2000+self.origin[1])
##        ax.set_zlim3d(-1000+self.origin[2], 2000+self.origin[2])
        ax.text2D(0.05, 0.95, 'frame: '+ str(frame_num), transform = self.ax.transAxes)
        plt.show()

    def _plot_frame(self, frame_num):
        self.ax.cla()
        for item in self.skeleton:
            tmp = [self.worldpos[frame_num][item[0]], self.worldpos[frame_num][item[1]]]
            tmp = np.transpose(np.array(tmp))
            self.ax.plot(*tmp, color = 'b')
##        self.ax.set_xlim3d(-4000+self.origin[0], 4000 + self.origin[0])
##        self.ax.set_ylim3d(-3000+self.origin[1], 3000+self.origin[1])
##        self.ax.set_zlim3d(-1000+self.origin[2], 3000+self.origin[2])
##        self.ax.set_xlim3d(self.xmin + self.origin[0], self.xmax + self.origin[0])
##        self.ax.set_ylim3d(self.ymin + self.origin[1], self.ymax + self.origin[1])
##        self.ax.set_zlim3d(self.zmin + self.origin[2], self.zmax + self.origin[2])
        #self.ax.set_aspect('equal')

  
        #self.ax.auto_scale_xyz([self.xmin-xoffset,self.xmax+xoffset], [self.ymin-yoffset, self.ymax+yoffset], [self.zmin, self.zmax+zoffset*2])
        self.ax.auto_scale_xyz([self.xmin,self.xmax], [self.ymin, self.ymax], [self.zmin, self.zmax])
        if self.filename != None:
            
            self.ax.set_title(self.filename)
        self.ax.text2D(0.05, 0.95, 'frame: '+ str(frame_num), transform = self.ax.transAxes)

    def animate_all(self):
        animation_range = [0,self.frameLen, 3]
        line_ani = animation.FuncAnimation(self.fig,
                                           self._plot_frame,
                                           np.arange(*animation_range),
                                           interval = 10,
                                           repeat = True)
        #line_ani.save('walking.mp4')
        plt.show()
        
    def animate_scene(self, animation_range):
        animation_range = [animation_range[0],animation_range[1], 1]
        line_ani = animation.FuncAnimation(self.fig,
                                           self._plot_frame,
                                           np.arange(*animation_range),
                                           interval = 50,
                                           repeat = True)
        
        plt.show()

    def plotFramesRange(self, plot_range):
        self.ax.cla()
        origin = self.worldpos[plot_range[0]]['root']
        for index in range(plot_range[0], plot_range[1],3):
            for item in self.skeleton:
                tmp = [self.worldpos[index][item[0]], self.worldpos[index][item[1]]]
                tmp = np.transpose(np.array(tmp))
                self.ax.plot(*tmp, color = 'b')
        self.ax.set_xlim3d(-4000+origin[0], origin[0])
        self.ax.set_ylim3d(-3000+origin[1], 1000+origin[1])
        self.ax.set_zlim3d(-1000+origin[2], 2000+origin[2])
        plt.show()

    def plotFrames(self, frames, color = 'blue'):
        ## display selected frames
        ## input: frames: a list contains frame indexes
        self.ax.cla()
        origin = self.worldpos[frames[0]]['root']
        for frame in frames:
            for item in self.skeleton:
                tmp = [self.worldpos[frame][item[0]], self.worldpos[frame][item[1]]]
                tmp = np.transpose(np.array(tmp))
                self.ax.plot(*tmp, color = color)
        self.ax.set_xlim3d(-2000+origin[0], 2000+origin[0])
        self.ax.set_ylim3d(-2000+origin[1], 2000+origin[1])
        self.ax.set_zlim3d(-1000+origin[2], 2000+origin[2])
        plt.show()        
