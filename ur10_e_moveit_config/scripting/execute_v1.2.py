#! /usr/bin/python
import os
import numpy as np
import math 



n_z = 1
n_x = 2
n_y = 2
brick_z = 0.04
brick_x = 0.2
brick_y = 0.1
i = 0
j = 0
k = 0


def print_cao(n_y,n_x,n_z,a,b,c,file):
	file.write("V1"+'\n'+'################################################'+'\n')
    for l in range(n_z-c):
        if (l == 0):
            for m in range (n_x-b):
                if(m == 0):
                    for n in range (n_y-a):
                        file.write('load("./brick.cao", t=['+str(m*brick_x)+';'+ str(n*brick_y)+';'+str(l*-1*brick_z)+'])'+'\n') ##load from the directory u placed brick.cao
                else:
                    for n in range (n_y):
                        file.write('load("./brick.cao", t=['+str((m)*brick_x)+';'+ str((n-a)*brick_y)+';'+str(l*-1*brick_z)+'])'+'\n')
        else:
            for m in range (n_x):
                for n in range (n_y):
                    file.write('load("./brick.cao", t=['+str((m-b)*brick_x)+';'+ str((n-a)*brick_y)+';'+str(l*-1*brick_z)+'])'+'\n')
    file.write('################################################'+'\n')
    file.write('# 3D points\n0                    # No 3D points\n# 3D lines\n0                    # No 3D lines\n# 3D faces from lines\n0                    # No 3D faces from lines\n# 3D faces from points\n0                    # No 3D faces from points\n# 3D cylinders\n0                    # No 3D cylinders\n# 3D circle\n0                    # No 3D circle')
	

for k in range (n_z):
    for j in range (n_x):
        for i in range (n_y):
            if(i==0 and j==0 and k==0):
                file = open("/home/civil/ur10e_paving/src/fmauch_universal_robot/ur10_e_moveit_config/scripting/objinitpose.out","r")
                pose = file.readlines()
                xcam_brick = float(pose[0].strip())
                ycam_brick = float(pose[1].strip())
                zcam_brick = float(pose[2].strip())
                rxcam_brick = float(pose[3].strip())
                rycam_brick = float(pose[4].strip())
                rzcam_brick = float(pose[5].strip())
                orixcam_brick = float(pose[6].strip())
                oriycam_brick = float(pose[7].strip())
                orizcam_brick = float(pose[8].strip())

                file.close()
                file = open("/home/civil/visp-ws/visp-build/tutorial/tracking/model-based/generic-rgbd/model/brick/brickstack/brick-stack.0.pos","w")
                file.write(str(float(xcam_brick))+'\n'+str(float(ycam_brick))+'\n'+str(float(zcam_brick))+'\n'+str(orixcam_brick)+'\n'+str(oriycam_brick)+'\n'+str(orizcam_brick))
                file.close()

                file = open("/home/civil/visp-ws/visp-build/tutorial/tracking/model-based/generic-rgbd/model/brick/brickstack/brick-stack.cao","w")
                print_cao(n_y,n_x,n_z,i,j,k,file)
                file.close()
                operation.readfile(operation.object_target)

            else:
                file = open("/home/civil/ur10e_paving/src/fmauch_universal_robot/ur10_e_moveit_config/scripting/objpose.out","r")
                pose = file.readlines()
                xcam_brick = float(pose[0].strip())
                ycam_brick = float(pose[1].strip())
                zcam_brick = float(pose[2].strip())
                rxcam_brick = float(pose[3].strip())
                rycam_brick = float(pose[4].strip())
                rzcam_brick = float(pose[5].strip())
                orixcam_brick = float(pose[6].strip())
                oriycam_brick = float(pose[7].strip())
                orizcam_brick = float(pose[8].strip())
                file.close()

                file = open("/home/civil/ur10e_paving/src/fmauch_universal_robot/ur10_e_moveit_config/scripting/matrix.out","r")
                pose1 = file.readlines()
                
                aa = float(pose1[0].strip())
                ab = float(pose1[1].strip())
                ac = float(pose1[2].strip())
                ba = float(pose1[3].strip())
                bb = float(pose1[4].strip())
                bc = float(pose1[5].strip())
                ca = float(pose1[6].strip())
                cb = float(pose1[7].strip())
                cc = float(pose1[8].strip())
                m = np.matrix([[aa,ab,ac],[ba,bb,bc],[ca,cb,cc]])
                file.close()

                cx = math.cos(rxcam_brick)
                sx = math.sin(rxcam_brick)
                cy = math.cos(rycam_brick)
                sy = math.sin(rycam_brick)
                cz = math.cos(rzcam_brick)
                sz = math.sin(rzcam_brick)
                Rx = np.matrix([[1,0,0],[0,cx,sx],[0,-sx,cx]])
                Ry = np.matrix([[cy,0,sy],[0,1,0],[sy,0,cy]])           
                Rz = np.matrix([[cz,sz,0],[-sz,cz,0],[0,0,1]])
                if (i==0):
                    if (j==0):
                        t_in_brick = [[-(n_x-1)*brick_x],[-(n_y-1)*brick_y],[-brick_z]]
                    else:
                        t_in_brick = [[brick_x],[-(n_y-1)*brick_y],[0.0000]]
                else:
                    t_in_brick = [[0.0000],[brick_y],[0.0000]]

                t_brick_to_cam = [[xcam_brick],[ycam_brick],[zcam_brick]]
                transformed_pose = m*t_in_brick+t_brick_to_cam

                file = open("/home/civil/visp-ws/visp-build/tutorial/tracking/model-based/generic-rgbd/model/brick/brickstack/brick-stack.0.pos","w")
                file.write(str(float(transformed_pose[0]))+'\n'+str(float(transformed_pose[1]))+'\n'+str(float(transformed_pose[2]))+'\n'+str(orixcam_brick)+'\n'+str(oriycam_brick)+'\n'+str(orizcam_brick))
                file.close()

                file = open("/home/civil/visp-ws/visp-build/tutorial/tracking/model-based/generic-rgbd/model/brick/brickstack/brick-stack.cao","w")
                print_cao(n_y,n_x,n_z,i,j,k,file)
                file.close()


            if(i==0 and j==0 and k==0):
                os.system("cp /home/civil/ur10e_paving/src/fmauch_universal_robot/ur10_e_moveit_config/scripting/objpose.out /home/civil/ur10e_paving/src/fmauch_universal_robot/ur10_e_moveit_config/scripting/objinitpose.out")



			#cmd = './tutorial-mb-generic-tracker-rgbd-realsense --model_color model/brick/brick40stack01/brick-stack.cao'
			#i += 1
		#j += 1
	#k += 1

	
