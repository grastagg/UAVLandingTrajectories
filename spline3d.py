import numpy as np
from scipy.interpolate import BSpline
import matplotlib.pyplot as plt

from pyoptsparse import Optimization, OPT

from Spherical_Object import Spherical_Object

def create_spline(t0,tf,c,k,p0,pf,v0,vf):
    '''
    :param t0: start and stop time of the spline (t0,t1)
    :param c: intermediate control points of the spline (3d controls points with shape (num_control_points-2,3) first and last control points are p0 and pf
    :param k: order of the spline
    :param p0: starting point of the spline (x,y,z)
    :param pf: ending point of the spline (x,y,z)
    :param v0: starting velocity of the spline (vx,vy,vz)
    :param vf: ennding velocity of the spline (vx,vy,vz)
    :return: scipy spline class with initial and final conditions set by parameters
    '''

    #### calculate the knot points of the spline ####

    l = len(c) + 4 #number of control points is the number of intermediate control points plus the number of fixed control points


    t=np.linspace(t0,tf,l-k+1,endpoint=True)

    #initial knots for clamped bslpine
    t_0 = t0*np.ones(k)

    t=np.append(t_0, t)

    #final knots for clamped bspline
    t_f = tf * np.ones(k)
    t=np.append(t, t_f)

    #calculate the second control point
    c1 = v0 * t[k+1]/k + p0

    #calculate the second to last control point
    c_sl = -vf * (tf - t[len(t) - k  -2]) / k + pf
    # print(c_sl)



    #stack all control points
    c_all = np.vstack((p0,c1,c,c_sl,pf))
    # print(c_all)

    spline = BSpline(t,c_all,k)
    return spline


def plot_spline(spl,num_samples,ax):
    control_points = spl.c
    t0 = spl.t[0]
    tf = spl.t[-1]
    t = np.linspace(t0,tf,num_samples,endpoint=True)
    out = spl(t)

    X = out[:,0]
    Y = out[:, 1]
    Z = out[:, 2]



    max_range = np.array([X.max() - X.min(), Y.max() - Y.min(), Z.max() - Z.min()]).max() / 2.0

    mid_x = (X.max() + X.min()) * 0.5
    mid_y = (Y.max() + Y.min()) * 0.5
    mid_z = (Z.max() + Z.min()) * 0.5
    ax.set_xlim(mid_x - max_range, mid_x + max_range)
    ax.set_ylim(mid_y - max_range, mid_y + max_range)
    ax.set_zlim(mid_z - max_range, mid_z + max_range)



    ax.plot3D(control_points[:, 0], control_points[:, 1], control_points[:, 2], 'k--', label='Control polygon', marker='o',
             markerfacecolor='red')
    ax.plot3D(out[:,0], out[:,1],out[:,2],c = 'lime')
    # ax.set_aspect('equal')
    plt.xlabel("x")
    plt.ylabel("y")


def plot_spherical_obsticles(obsticles, ax):
    for i in range(len(obsticles)):
        u = np.linspace(0, np.pi, 30)
        v = np.linspace(0, 2 * np.pi, 30)
        radius = obsticles[i].radius
        center_x = obsticles[i].center_x
        center_y = obsticles[i].center_y
        center_z = obsticles[i].center_z
        x = radius * np.outer(np.sin(u), np.sin(v)) + center_x
        y = radius * np.outer(np.sin(u), np.cos(v)) + center_y
        z = radius * np.outer(np.cos(u), np.ones_like(v)) + center_z
        ax.plot_surface(x, y, z)


def get_spline_dist(spl,num_points):
    t0 = spl.t[0]
    tf = spl.t[-1]
    t = np.linspace(t0, tf, num_points, endpoint=True)
    x = spl(t)[:, 0]

    y = spl(t)[:, 1]
    z = spl(t)[:, 2]

    total_dist = 0
    for i in range(len(x) - 1):
        dist = np.sqrt((x[i] - x[i + 1]) ** 2 + (y[i] - y[i + 1]) ** 2 + (z[i] - z[i + 1])**2)
        total_dist += dist
    return total_dist

def spherical_obstalce(obstacles,spl_p):

    x = spl_p[:,0]
    y = spl_p[:,1]
    z = spl_p[:,1]
    dist = np.zeros(len(x)*len(obstacles))
    index = 0
    for j in range(len(obstacles)):
        center_x = obstacles[j].center_x
        center_y = obstacles[j].center_y
        center_z = obstacles[j].center_z
        radius = obstacles[j].radius
        for i in range(len(z)):
            ##something isnt working here
            dist[index] = np.sqrt((x[i]-center_x)**2+(y[i]-center_y)**2+(z[i]-center_z)**2) - 2*radius
            index += 1

    # print("x",x
    # print("y",y)
    # print("z",z)
    return dist

def min_func_spline_trajectory(p0, p1,v0,vf,k, num_cont_points, lb, ub, v_lower_limit,
                              v_upper_limit,a_lower_limit,a_upper_limit,t0,tf_initial,num_samples,obstacles):
    '''
    :param p0: initial position of the vehicle (x,y,z)
    :param p1: desired landing position of the vehicle (x,y,z)
    :param v0: initial velocity of the vehicle (vx,vy,vz)
    :param vf: landing velocity of the vehicle (vx,vy,vz)
    :param k: order of the b-spline trajectory
    :param num_cont_points: number of control points (4 will be fixed by initial and final conditions)
    :param lb: lower bound of control point location (set to level of ground to ensure trajecotory does not go beneath surface FIX THIS!!!
    :param ub: upper bound of control point location
    :param v_lower_limit: minimum norm of the velocity of the vehicle (i.e. make sure above stall speed for fixed wing)
    :param v_upper_limit: maximum norm of the velocity of vehicle
    :param a_lower_limit: Minimum norm of acceleration of the vehicle
    :param a_upper_limit: Maximum norm of acceleration of the vehicle
    :param t0: start time of the spline (usually set to 0)
    :param tf_initial: initial condition for end time of spline
    :param num_samples: number of discrete points on spline to check acceleration and velocity
    :return: optimal spline trajectory
    '''

    def objfunc(xdict):
        '''
        :param xdict: dictionary of control variables for the optimizer
        :return: funcs: dictionary of objective and constraints, fail: whether function could be run
        '''
        nonlocal t0,k,p0,p1,v0,vf,num_samples,obstacles

        #extract control points and final time from input dict
        x = xdict["control points"]
        tf = xdict["tf"][0]


        funcs = {}

        #reshape control points
        x = x.reshape(num_cont_points - 4, len(p0))

        #create spline from control points and intitial and final conditions
        spl = create_spline(t0,tf,x,k,p0,p1,v0,vf)

        #create discrete set of times to sample spline at
        t = np.linspace(t0, tf, num_samples, endpoint=True)


        spl_p = spl(t)

        #evaluate the first derivative of the spline
        spl_d = spl.derivative(1)(t)

        #extract derivative in all directions
        x_dot = spl_d[:,0]
        y_dot = spl_d[:,1]
        z_dot = spl_d[:,2]

        # evaluate the first derivative of the spline
        spl_dd = spl.derivative(2)(t)
        x_ddot = spl_dd[:,0]
        y_ddot = spl_dd[:,1]
        z_ddot = spl_dd[:,2]

        #euclidean norm of the velocity
        v = np.sqrt(np.square(x_dot) + np.square(y_dot) + np.square(z_dot))

        #euclidean norm of the accelerations
        a = np.sqrt(np.square(x_ddot) + np.square(y_ddot) + np.square(z_ddot))

        #calculate the distance of the spline
        dist = get_spline_dist(spl, num_samples)

        #check if path is outside obsticales
        obsticle_dist = spherical_obstalce(obstacles,spl_p)

        #add objective and constraints to funcs dict
        funcs['dist'] = dist
        funcs["time"] = tf
        funcs["velocity"] = v
        funcs['acceleration'] = a
        funcs['obstacle'] = obsticle_dist

        fail = False

        return funcs, fail



##fix this trying to create evenely spaced initial intermediate control points
    x0 = np.zeros(len(p0) * (num_cont_points - 4))
    dx = (p1[0] - p0[0]) / (num_cont_points - 3)
    dy = (p1[1] - p0[1]) / (num_cont_points - 3)
    dz = (p1[2] - p0[2]) / (num_cont_points - 3)
    x_0 = np.linspace(p0[0] + dx, p1[0] - dx, (num_cont_points - 4))
    y0 = np.linspace(p0[1] + dy, p1[1] - dy, (num_cont_points - 4))
    z0 = np.linspace(p0[2] + dz, p1[2] - dz, (num_cont_points - 4))
    ind = 0
    for i in range(len(x_0)):
        x0[ind] = x_0[i]
        ind += 1
        x0[ind] = y0[i]
        ind += 1
        x0[ind] = z0[i]
        ind += 1

    print("x0", x0)

    # create lower bound for control points, optimizer sorts control points into x as [x1,y1,z1,x2,y2,z2,...
    # so every third element of the lower bound needs to be 0 to ensure trajectory does not cross below ground
    low_b = np.zeros(len(x0))
    for i in range(len(low_b)):
        #check if it is not the third,sixth,... element
        if not (i+1) % 3 == 0:

            low_b[i] = lb

    print(lb)
    optProb = Optimization("B-spline landing trajectory", objfunc)

    optProb.addVarGroup(name="control points", nVars=len(p0) * (num_cont_points - 4), varType="c", value=x0, lower=low_b,
                        upper=ub)
    optProb.addVarGroup(name="tf", nVars=1, varType="c", value=tf_initial, lower=0,upper=None)

    optProb.addConGroup("velocity", num_samples, lower=v_lower_limit, upper=v_upper_limit, scale=1.0 / v_upper_limit)
    optProb.addConGroup("acceleration", num_samples, lower=a_lower_limit, upper=a_upper_limit, scale=1.0 / a_upper_limit)
    optProb.addConGroup("obstacle", num_samples*len(obstacles), lower=0, upper=None)



    optProb.addObj("time")

    opt = OPT("ipopt")

    sol = opt(optProb, sens="FD")

    print(sol)
    c = sol.xStar['control points'].reshape(num_cont_points - 4, len(p0))
    spl = create_spline(t0,sol.xStar['tf'],c,k,p0,p1,v0,vf)

    return spl

def plot_velocity_and_acceleration(spl,v_lower_limit,v_upper_limit,a_lower_limit,a_upper_limit,num_samples):
    t0 = spl.t[0]
    tf = spl.t[-1]

    t = np.linspace(t0,tf,num_samples,endpoint=True)

    x_dot = spl.derivative(1)(t)[:, 0]
    y_dot = spl.derivative(1)(t)[:, 1]
    z_dot = spl.derivative(1)(t)[:, 2]

    x_ddot = spl.derivative(2)(t)[:, 0]
    y_ddot = spl.derivative(2)(t)[:, 1]
    z_ddot = spl.derivative(2)(t)[:, 2]

    # norm of the velocity (directionless velocity)
    v = np.sqrt(np.square(x_dot) + np.square(y_dot) + np.square(z_dot))

    # norm of the accelerations
    a = np.sqrt(np.square(x_ddot) + np.square(y_ddot) + np.square(z_ddot))


    fig, axs = plt.subplots(2)
    axs[0].plot(t,v,c='b')
    axs[0].plot(t, v_lower_limit*np.ones(len(t)),c='r')
    axs[0].plot(t, v_upper_limit * np.ones(len(t)),c='r')
    axs[0].title.set_text("Velocity")
    axs[1].plot(t, a,c='b')
    axs[1].plot(t, a_lower_limit*np.ones(len(t)),c='r')
    axs[1].plot(t, a_upper_limit * np.ones(len(t)),c='r')
    axs[1].title.set_text("Acceraltion")
    plt.tight_layout()
    plt.show()

def get_random_spherical_obsticales(num_obst,x_lim,y_lim,z_lim,min_radius,max_radius):
    '''
    :param num_obst:
    :param x_lim:
    :param y_lim:
    :param z_lim:
    :param min_radius:
    :param max_radius:
    :return:
    '''

    obsticles = []

    for i in range(num_obst):
        center_x = np.random.uniform(x_lim[0],x_lim[1],1)
        center_y = np.random.uniform(y_lim[0],y_lim[1],1)
        center_z = np.random.uniform((z_lim[0]),z_lim[1],1)
        radius = np.random.uniform(min_radius,max_radius,1)
        tmp_sphere = Spherical_Object((center_x,center_y,center_z),radius)
        obsticles.append(tmp_sphere)

    return obsticles


def main():

    k = 3

    pf = np.array([0,0,0])
    p0 = np.array([0,0,10])

    v0 = np.array([0,-2,0])
    vf =np.array([0,-5,-.1])

    num_cont_points = 7

    lb = -100
    ub = 100
    v_lower_limit = 2
    v_upper_limit = 100

    a_lower_limit = 0
    a_upper_limit = 20
    t0 = 0
    tf_initial = 1

    num_samples = 20

    #list of obstacles
    num_obst = 0
    x_lim= [-.1,.1]
    y_lim = [-.1,.1]
    z_lim = [3,6]
    min_radius = 1
    max_radius = 3
    obstacles = get_random_spherical_obsticales(num_obst, x_lim, y_lim, z_lim, min_radius, max_radius)
    print(obstacles)


    spl = min_func_spline_trajectory(p0, pf, v0, vf, k, num_cont_points, lb, ub, v_lower_limit,
                              v_upper_limit, a_lower_limit, a_upper_limit, t0, tf_initial, num_samples,obstacles)

    ax = plt.axes(projection='3d')
    plot_spline(spl,100,ax)
    plot_spherical_obsticles(obstacles,ax)
    plot_velocity_and_acceleration(spl,v_lower_limit,v_upper_limit,a_lower_limit,a_upper_limit,num_samples = 100)

    print(get_spline_dist(spl, num_samples))

if __name__ == "__main__":

    main()