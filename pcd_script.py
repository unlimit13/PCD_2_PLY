import os
import open3d as o3d


def distance_check(X,Y,Z):
    distance = float(Z)
    #distance = ((float(X)*10)**2 + (10*float(Y))**2 + (10*float(Z))**2)**(1/2)
    return distance


if os.system("cp A.pcd Edited_A.pcd")==0:
    new_file = open("./Edited_A.pcd", 'w')
    old_file = open("./A.pcd", 'r')


for i in range(0,11):
    line = old_file.readline()
    new_file.write(line)

cnt=0
while line:
    line = old_file.readline()
    XYZ= line.split()
    
    if len(line)==0 or distance_check(XYZ[0],XYZ[1],XYZ[2]) >1.8:
        continue
    else :
        new_file.write(line)
        cnt+=1

    
new_file.close()
old_file.close()
new_file = open("./Edited_A.pcd", "r")
lines_all = new_file.readlines()
new_file_w = open("./Edited_A.pcd", "w")


new_string_7 = "WIDTH "+ str(cnt) + "\n"
new_string_10 = "POINTS "+ str(cnt) + "\n"


lines=[]

for i,l in enumerate(lines_all):
    if i==6:
        lines= lines+[new_string_7]
    elif i==9:
        lines= lines+[new_string_10]
    else:
        lines= lines+[l]
new_file_w.writelines(lines)
new_file_w.close()
new_file.close()
    
    
pcd = o3d.io.read_point_cloud("Edited_A.pcd")

alpha = 0.0136
print(f"alpha={alpha:.3f}")
mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_alpha_shape(pcd, alpha)
mesh.compute_vertex_normals()
o3d.visualization.draw_geometries([mesh], mesh_show_back_face=True)#to ply
o3d.io.write_triangle_mesh("EEE.ply",mesh)

'''alpha = 0.05
pcd = o3d.io.read_point_cloud("Edited_A.pcd")
mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_alpha_shape(pcd, alpha)
mesh.compute_vertex_normals()
pcd = mesh.sample_points_poisson_disk(3000)
o3d.visualization.draw_geometries([pcd])
radii = [0.005, 0.01, 0.02, 0.04]
rec_mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_ball_pivoting(
    pcd, o3d.utility.DoubleVector(radii))
o3d.visualization.draw_geometries([pcd, rec_mesh])''' #to ply

'''pcd = o3d.io.read_point_cloud("Edited_A.pcd")
print(pcd)
o3d.visualization.draw_geometries([pcd],
                                  zoom=0.664,
                                  front=[-0.4761, -0.4698, -0.7434],
                                  lookat=[1.8900, 3.2596, 0.9284],
                                  up=[0.2304, -0.8825, 0.4101])
print('run Poisson surface reconstruction')
with o3d.utility.VerbosityContextManager(
        o3d.utility.VerbosityLevel.Debug) as cm:
    mesh, densities = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(
        pcd, depth=9)
print(mesh)
o3d.visualization.draw_geometries([mesh],
                                  zoom=0.664,
                                  front=[-0.4761, -0.4698, -0.7434],
                                  lookat=[1.8900, 3.2596, 0.9284],
                                  up=[0.2304, -0.8825, 0.4101])''' #reconstruction


