# Deployable Func Rotation
import numpy as np
import math as m


def RotationMatrixEuler(z, y, x):
  alpha = m.radians(x)      # x axis rotation angle
  theta = m.radians(y)      # y axis rotation angle
  gamma = m.radians(z)      # z axis rotation angle


  euler_angle = [alpha, theta, gamma]

  rotM_x = np.array([   [1, 0, 0], 
                        [ 0,    m.cos(euler_angle[0]),   -m.sin(euler_angle[0])], 
                        [ 0,    m.sin(euler_angle[0]),   m.cos(euler_angle[0])]   ]   )


  rotM_y =  np.array([   [ m.cos(euler_angle[1]),  0,  m.sin(euler_angle[1])],
                        [ 0,               1,      0],
                        [-m.sin(euler_angle[1]),   0,      m.cos(euler_angle[1])] ]  )


  rotM_z = np.array([   [m.cos(euler_angle[2]),    -m.sin(euler_angle[2]),    0],
                      [m.sin(euler_angle[2]),    m.cos(euler_angle[2]),     0],
                      [0,                     0,                      1]
                      ])

  R = np.dot(rotM_z, np.dot(rotM_y, rotM_x ))
  return R

#Deployable Func : entry == Rotation Matrix and the relative orientation vector ({O} ---> {A}) 
def mount_HomogeneousMT(rotM, o_p_a):
  homo = rotM
  add_column = np.array(o_p_a)
  add_lastLine = np.array([0,0,0,1])
  result = np.column_stack((homo, add_column))
  result = np.vstack((result, add_lastLine))
  return result

def adjoint(A):
  """compute inverse without division by det; ...xv3xc3 input, or array of matrices assumed"""
  AI = np.linalg.inv(A)
  return AI



## ------------ https://learnopencv.com/rotation-matrix-to-euler-angles/ -------------- ##
# Checks if a matrix is a valid rotation matrix.
def isRotationMatrix(R) :
    Rt = np.transpose(R)
    shouldBeIdentity = np.dot(Rt, R)
    I = np.identity(3, dtype = R.dtype)
    n = np.linalg.norm(I - shouldBeIdentity)
    return n < 1e-6
 
# Calculates rotation matrix to euler angles
# The result is the same as MATLAB except the order
# of the euler angles ( x and z are swapped ).
def rotationMatrixToEulerAngles(R) :
 
    assert(isRotationMatrix(R))
 
    sy = m.sqrt(R[0,0] * R[0,0] +  R[1,0] * R[1,0])
 
    singular = sy < 1e-6
 
    if  not singular :
        x = m.atan2(R[2,1] , R[2,2])
        y = m.atan2(-R[2,0], sy)
        z = m.atan2(R[1,0], R[0,0])
    else :
        x = m.atan2(-R[1,2], R[1,1])
        y = m.atan2(-R[2,0], sy)
        z = 0
 
    return np.array([x, y, z])