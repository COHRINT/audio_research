'''
***********************************************************
File: gaussianMixtures.py
Classes: GM,Gaussian

Allows for the creation, use, and compression of mixtures
of multivariate normals, or Gaussian Mixture Models (GMM).



***********************************************************
'''


__author__ = "Jeremy Muesing"
__copyright__ = "Copyright 2017, Cohrint"
__credits__ = ["Jeremy Muesing", "Luke Burks", "Nisar Ahmed"]
__license__ = "GPL"
__version__ = "1.0.8"
__maintainer__ = "Jeremy Muesing"
__email__ = "jeremy.muesing@colorado.edu"
__status__ = "Development"


from gaussianMixtures import GM
from gaussianMixtures import Gaussian
from scipy.stats import multivariate_normal as mvn
import numpy as np
import math
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import matplotlib.image as mgimg
import rospy
from std_msgs.msg import Float64

class Bayesian:
	def __init__(self):
		self.transition_model()
		self.image_list=[]
		self.fig,self.ax=plt.subplots()

	def belief_update(self,prior,robot_x,robot_y,robot_theta,bearing):
		post=GM()
		obs_model=self.observation_model(robot_x,robot_y,robot_theta,theta)
		# x,y,c=obs_model[0].plot2D(low = [-10,0],high = [10,10],xlabel='x position (m)',ylabel='y position (m)',vis=False)
		# plt.contourf(x,y,c,cmap='inferno')
		# plt.show()
		for i in prior:
			for j in obs_model[0]:
				mean_i=np.matrix(i.mean)
				mean_j=np.matrix(j.mean)
				var_i=np.matrix(i.var)
				var_j=np.matrix(j.var)
				var_a=np.matrix(self.sigma_a)
				del_a=np.matrix(self.delta_a)
				arg2 = mean_i+del_a
				arg3 = var_j+var_a+var_i

				w_k=i.weight*j.weight*mvn.pdf(mean_j.tolist()[0],arg2.tolist()[0],arg3)

				C2=(var_j.I + (var_a+var_i).I).I
				C1=C2*(var_j.I*mean_j.T+(var_a+var_i).I*(mean_i+del_a).T)

				post.addNewG(C1.T.tolist()[0],C2.tolist(),w_k)
		post.normalizeWeights()
		return post

	def observation_model(self,robot_x,robot_y,robot_theta,bearing):
		obs_model = []
		obs_model.append(GM())
		for r in range(15):
			x=robot_x+r*math.cos(math.radians(robot_theta+(360-bearing)+90))
			y=robot_y+r*math.sin(math.radians(robot_theta+(360-bearing)+90))
			scalar = math.sqrt(r)+1
			obs_model[0].addNewG([x,y],[[scalar,0],[0,scalar]],scalar**3.2)
		return obs_model

	def transition_model(self):
		self.delta_a = [0,0]
		scale=2
		self.sigma_a = [[1,0],[0,1]]

	def update(self,prior,robot_x,robot_y,robot_theta,bearing):
		post=self.belief_update(prior,robot_x,robot_y,robot_theta,bearing)
		post=post.kmeansCondensationN(k=10)
		# post.plot2D(low = [-10,0],high = [10,10],xlabel='x position (m)',ylabel='y position (m)')
		# self.continue_plot(post,robot_x,robot_y,robot_theta,bearing)
		# print post.size
		return post

	def continue_plot(self,gm,robot_x,robot_y,robot_theta,bearing,t):
		x,y,c=gm.plot2D(low=[-10,0],high=[10,10],vis=False)
		self.ax.cla()
		self.ax.contourf(x,y,c)
		self.ax.set_title('time step='+str(t))
		self.ax.set_xlabel('position (m)')
		self.ax.set_ylabel('position (m)')
		robot=self.ax.plot([robot_x], [robot_y], 'go')
		r=1*np.random.randn()+6
		other,=self.ax.plot([robot_x+r*math.cos(math.radians(robot_theta+(360-bearing)+90))],
						[robot_y+r*math.sin(math.radians(robot_theta+(360-bearing)+90))],
						'ro')
		self.fig.savefig('../tmp/img'+str(t)+".png",bbox_inches='tight',pad_inches=0)
		plt.pause(.5)
		other.remove()

def listner():
	rospy.init_node('listner',anonymous=True)
	theta=rospy.wait_for_message('local_sound',Float64,timeout=15)
	return theta.data
	rospy.spin()



if __name__ == '__main__':
	a=Bayesian()

	# all_of_it=GM()
	# for i in range(360):
	# 	all_of_it.addGM(a.obs_model[i])
	# x,y,c=all_of_it.plot2D(low = [-10,-10],high = [10,10],vis=False)
	# c=c/np.amax(c)
	# plt.contourf(x,y,c)
	# plt.colorbar()
	# plt.show()

	robot_position=[0,0,0]
	theta=listner()
	belief=GM()
	belief.addNewG([0,3],[[5,0],[0,5]],1)
	counter=1
	# belief.plot2D(low=[-10,0],high=[10,10],xlabel='x position (m)',ylabel='y position (m)')
	a.continue_plot(belief,robot_position[0],robot_position[1],robot_position[2],int(round(theta)),counter)
	counter=counter+1
	while (counter < 10):
		theta=listner()
		belief=a.update(belief,robot_position[0],robot_position[1],robot_position[2],int(round(theta)))
		a.continue_plot(belief,robot_position[0],robot_position[1],robot_position[2],int(round(theta)),counter)
		counter=counter+1

	fig,ax=plt.subplots()
	images=[]
	for k in range(1,counter):
		fname='../tmp/img%d.png' %k
		img=mgimg.imread(fname)
		imgplot=plt.imshow(img)
		plt.axis('off')
		images.append([imgplot])
	ani=animation.ArtistAnimation(fig,images,interval=20)
	ani.save("../animation.gif",fps=1,writer='animation.writer')
