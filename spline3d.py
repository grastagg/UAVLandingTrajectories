import numpy as np
from scipy.interpolate import BSpline
import matplotlib.pyplot as plt

from pyoptsparse import Optimization, OPT

from Spherical_Object import Spherical_Object

from scipy.optimize import minimize

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

def plot_splines(splines,num_samples,ax):
    colors = ['lime','magenta','blue']

    for i in range(len(splines)):
        spl = splines[i]
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
        ax.plot3D(out[:,0], out[:,1],out[:,2],c = colors[i],label = "agent " + str(i))
    # ax.set_aspect('equal')
    plt.legend
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
            dist[index] = np.sqrt((x[i]-center_x)**2+(y[i]-center_y)**2+(z[i]-center_z)**2) - radius
            index += 1

    # print("x",x
    # print("y",y)
    # print("z",z)
    return dist

def min_func_spline_trajectory(p0, pf,v0,vf,k, num_cont_points, lb, ub, v_lower_limit,
                              v_upper_limit,thrust_lower_limit,thrust_upper_limit,roll_angle_lower_limit,roll_angle_upper_limit,t0,tf_initial,num_samples,
                               obstacles,min_flight_path_angle,max_flight_path_angle,g,mass,F_drag):
    '''
    :param p0: initial position of the vehicle (x,y,z)
    :param pf: desired landing position of the vehicle (x,y,z)
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
    num_agents = len(p0)
    def objfunc(xdict):
        '''
        :param xdict: dictionary of control variables for the optimizer
        :return: funcs: dictionary of objective and constraints, fail: whether function could be run
        '''
        nonlocal t0,k,p0,pf,v0,vf,num_samples,obstacles,num_agents,g,mass,F_drag


        #extract control points and final time from input dict
        x = xdict["control points"]
        tf = xdict["tf"]


        funcs = {}

        #reshape control points






        v_all = np.array([])
        obsticle_all = np.array([])
        flight_path_angle_all = np.array([])
        roll_angle_all = np.array([])
        thrust_all = np.array([])
        # t_max = 0
        for i in range(num_agents):
            # create spline from control points and intitial and final conditions
            x_tmp = x[(num_cont_points - 4)*i:(num_cont_points - 4)*i + 3*(num_cont_points - 4)]
            x_tmp = x_tmp.reshape(num_cont_points - 4, len(p0[i]))
            tf_tmp = tf[i]
            spl = create_spline(t0, tf_tmp, x_tmp, k, p0[i], pf[i], v0[i], vf[i])
            # create discrete set of times to sample spline at
            t = np.linspace(t0, tf[i], num_samples, endpoint=True)

            spl_p = spl(t)

            #evaluate the first derivative of the spline
            spl_d = spl.derivative(1)(t)

            #extract derivative in all directions
            x_dot = spl_d[:,0]
            y_dot = spl_d[:,1]
            z_dot = spl_d[:,2]

            # evaluate the second derivative of the spline
            spl_dd = spl.derivative(2)(t)

            x_ddot = spl_dd[:,0]
            y_ddot = spl_dd[:,1]
            z_ddot = spl_dd[:,2]


            #x,y path angle
            phi = np.arctan2(x_dot,y_dot)

            #derivative of the x,y path angle
            phi_dot = (np.multiply(x_dot,y_ddot) - np.multiply(y_dot,x_ddot))/(.001 + np.square(x_dot) + np.square(y_dot))

            #### CONSTRAINTS ####

            #check if path is outside obsticales
            obsticle_dist = spherical_obstalce(obstacles,spl_p)
            obsticle_all = np.concatenate((obsticle_all,obsticle_dist))

            #euclidean norm of the velocity
            v = np.sqrt(np.square(x_dot) + np.square(y_dot) + np.square(z_dot))
            v_all = np.concatenate((v_all,v))

            #calculate flight path angle
            flight_path_angle = np.pi/2 - np.arctan2(np.sqrt(np.square(x_dot) + np.square(y_dot)) , z_dot)
            flight_path_angle_all = np.concatenate((flight_path_angle_all,flight_path_angle))

            #calculate the roll angle
            roll_angle = np.arctan2(np.multiply(v,phi_dot),g)
            roll_angle_all = np.concatenate((roll_angle_all, roll_angle))

            #calculate the thrust
            thrust = (np.cos(flight_path_angle)*np.cos(phi)*y_ddot + np.cos(flight_path_angle)*np.sin(phi)*x_ddot)*mass + F_drag
            thrust_all = np.concatenate((thrust_all, thrust))
            #


        #add objective and constraints to funcs dict
        # funcs['dist'] = dist
        # print(tf)
        funcs["time"] = np.sum(np.square(tf))
        # print(funcs['time'])
        funcs["velocity"] = v_all
        funcs['roll_angle'] = roll_angle_all
        funcs['thrust'] = thrust_all
        funcs['obstacle'] = obsticle_all
        funcs['flight_path_angle'] = flight_path_angle_all

        fail = False

        return funcs, fail



##fix this trying to create evenely spaced initial intermediate control points
    x0 = np.zeros(len(p0[0]) * (num_cont_points - 4) * num_agents)
    print("x0",x0.shape)
    ind = 0
    for i in range(num_agents):
        dx = (pf[i][0] - p0[i][0]) / (num_cont_points - 3)
        dy = (pf[i][1] - p0[i][1]) / (num_cont_points - 3)
        dz = (pf[i][2] - p0[i][2]) / (num_cont_points - 3)

        x_0 = np.linspace(p0[i][0] + dx, pf[i][0] - dx, (num_cont_points - 4))
        print(len(x_0))
        y0 = np.linspace(p0[i][1] + dy, pf[i][1] - dy, (num_cont_points - 4))
        z0 = np.linspace(p0[i][2] + dz, pf[i][2] - dz, (num_cont_points - 4))
        for j in range(len(x_0)):
            x0[ind] = x_0[j]
            ind += 1
            x0[ind] = y0[j]
            ind += 1
            x0[ind] = z0[j]
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

    optProb.addVarGroup(name="control points", nVars=num_agents * len(p0[0]) * (num_cont_points - 4), varType="c", value=x0, lower=low_b,
                        upper=ub)
    optProb.addVarGroup(name="tf", nVars=num_agents, varType="c", value=tf_initial, lower=0,upper=None)

    optProb.addConGroup("velocity", num_agents*num_samples, lower=v_lower_limit, upper=v_upper_limit, scale=1.0 / v_upper_limit)
    optProb.addConGroup("thrust", num_agents * num_samples, lower=thrust_lower_limit, upper=thrust_upper_limit, scale=1.0 / thrust_upper_limit)
    optProb.addConGroup("roll_angle", num_agents * num_samples, lower=roll_angle_lower_limit, upper=roll_angle_upper_limit, scale=1.0 / roll_angle_upper_limit)
    optProb.addConGroup("obstacle", num_agents * num_samples*len(obstacles), lower=0, upper=None)
    optProb.addConGroup("flight_path_angle", num_agents * num_samples, lower=min_flight_path_angle, upper=max_flight_path_angle)



    optProb.addObj("time")

    opt = OPT("ipopt")
    opt.options['print_level'] = 5

    sol = opt(optProb, sens="FD")

    print(sol)

    # create spline from control points and intitial and final conditions
    x = sol.xStar['control points']
    tf = sol.xStar['tf']
    splines = []
    for i in range(num_agents):
        x_tmp = x[(num_cont_points - 4) * i:(num_cont_points - 4) * i + 3 * (num_cont_points - 4)]
        x_tmp = x_tmp.reshape(num_cont_points - 4, len(p0[i]))
        tf_tmp = tf[i]
        spl = create_spline(t0, tf_tmp, x_tmp, k, p0[i], pf[i], v0[i], vf[i])
        splines.append(spl)






    # spl = create_spline(t0,sol.xStar['tf'][0],c,k,p0[0],pf[0],v0[0],vf[0])

    return splines


def min_func_spline_trajectory_gradient_free(p0, pf,v0,vf,k, num_cont_points, lb, ub, v_lower_limit,
                              v_upper_limit,a_lower_limit,a_upper_limit,j_lower_limit, j_upper_limit,t0,tf_initial,num_samples,obstacles,min_flight_path_angle,max_flight_path_angle):
    '''
    This is the function for gradient free optimization of our problem, the constraints are added as penelty terms
    :param p0: initial position of the vehicle (x,y,z)
    :param pf: desired landing position of the vehicle (x,y,z)
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

    def objfunc(x0):

        nonlocal t0, k, p0, pf, v0, vf, num_samples, obstacles, v_lower_limit, v_upper_limit, a_lower_limit, a_upper_limit,j_lower_limit, j_upper_limit, min_flight_path_angle, max_flight_path_angle

        mu = 100
        # extract control points and final time from input dict
        x = x0[0:-1]
        tf = x0[-1]

        # if tf < 0:
        #     return 10000

        funcs = {}

        # reshape control points
        x = x.reshape(num_cont_points - 4, len(p0))

        # create spline from control points and intitial and final conditions
        spl = create_spline(t0, tf, x, k, p0, pf, v0, vf)

        # create discrete set of times to sample spline at
        t = np.linspace(t0, tf, num_samples, endpoint=True)

        spl_p = spl(t)

        # evaluate the first derivative of the spline
        spl_d = spl.derivative(1)(t)

        # extract derivative in all directions
        x_dot = spl_d[:, 0]
        y_dot = spl_d[:, 1]
        z_dot = spl_d[:, 2]

        # evaluate the first derivative of the spline
        spl_dd = spl.derivative(2)(t)
        x_ddot = spl_dd[:, 0]
        y_ddot = spl_dd[:, 1]
        z_ddot = spl_dd[:, 2]

        #evaluate the third derivative of the spline
        spl_ddd = spl.derivative(3)(t)

        x_dddot = spl_ddd[:,0]
        y_dddot = spl_ddd[:,1]
        z_dddot = spl_ddd[:,2]

        # euclidean norm of the velocity
        v = np.sqrt(np.square(x_dot) + np.square(y_dot) + np.square(z_dot))

        # euclidean norm of the accelerations
        a = np.sqrt(np.square(x_ddot) + np.square(y_ddot) + np.square(z_ddot))

        j = np.sqrt(np.square(x_dddot) + np.square(y_dddot) + np.square(z_dddot))

        # calculate the decent angle
        flight_path_angle = np.pi / 2 - np.arctan2(np.sqrt(np.square(x_dot) + np.square(y_dot)), z_dot)

        #check if path is outside obsticales
        obsticle_dist = spherical_obstalce(obstacles,spl_p)

        # create the penalty part of the objective
        g1 = np.maximum(0, -v_upper_limit + v)
        g2 = np.maximum(0, v_lower_limit - v)
        g3 = np.maximum(0, -a_upper_limit + a)
        g4 = np.maximum(0, a_lower_limit - a)
        g5 = np.maximum(0, -max_flight_path_angle + flight_path_angle)
        g6 = np.maximum(0, min_flight_path_angle - flight_path_angle)
        g7 = np.maximum(0, -j_upper_limit + j)
        g8 = np.maximum(0, j_lower_limit - j)
        g9 = np.maximum(0, -obsticle_dist)

        # sum of squared penalties
        sum_g = np.sum(np.square(g1)) + np.sum(np.square(g2)) + np.sum(np.square(g3)) + np.sum(np.square(g4)) + np.sum(
            np.square(g5)) + np.sum(np.square(g6)) + np.sum(np.square(g7))+ np.sum(np.square(g8))+ np.sum(np.square(g9))

        # add objective and constraints to funcs dict
        obj = tf + mu * sum_g

        return obj



##fix this trying to create evenely spaced initial intermediate control points
    x0 = np.zeros(len(p0) * (num_cont_points - 4) + 1) #added 1 so I can add tf variable to end of vector
    dx = (pf[0] - p0[0]) / (num_cont_points - 3)
    dy = (pf[1] - p0[1]) / (num_cont_points - 3)
    dz = (pf[2] - p0[2]) / (num_cont_points - 3)
    x_0 = np.linspace(p0[0] + dx, pf[0] - dx, (num_cont_points - 4))
    y0 = np.linspace(p0[1] + dy, pf[1] - dy, (num_cont_points - 4))
    z0 = np.linspace(p0[2] + dz, pf[2] - dz, (num_cont_points - 4))
    ind = 0
    for i in range(len(x_0)-1):
        x0[ind] = x_0[i]
        ind += 1
        x0[ind] = y0[i]
        ind += 1
        x0[ind] = z0[i]
        ind += 1
    x0[-1] = tf_initial
    print("x0", x0)

    # create lower bound for control points, optimizer sorts control points into x as [x1,y1,z1,x2,y2,z2,...
    # so every third element of the lower bound needs to be 0 to ensure trajectory does not cross below ground
    low_b = np.zeros(len(x0))
    for i in range(len(low_b)):
        #check if it is not the third,sixth,... element
        if not (i+1) % 3 == 0:

            low_b[i] = lb

    options = {'maxiter': 100000}

    bounds = []
    for i in range(len(x0)-1):
        bounds.append((lb,ub))
    bounds.append((0,100))

    sol = minimize(objfunc, x0, method="Nelder-Mead",bounds = bounds, options=options)


    print(sol)
    c = sol.x[0:-1].reshape(num_cont_points - 4, len(p0))
    tf = sol.x[-1]
    spl = create_spline(t0, tf, c, k, p0, pf, v0, vf)
    return spl


def plot_constraints(splines,v_lower_limit,v_upper_limit,thrust_lower_limit,thrust_upper_limit,roll_angle_lower_limit,roll_angle_upper_limit,min_flight_path_angle,max_flight_path_angle,num_samples,g,mass,F_drag):
    for i in range(len(splines)):
        spl = splines[i]
        t0 = spl.t[0]
        tf = spl.t[-1]

        t = np.linspace(t0,tf,num_samples,endpoint=True)


        # evaluate the first derivative of the spline
        spl_d = spl.derivative(1)(t)

        # extract derivative in all directions
        x_dot = spl_d[:, 0]
        y_dot = spl_d[:, 1]
        z_dot = spl_d[:, 2]

        # evaluate the second derivative of the spline
        spl_dd = spl.derivative(2)(t)

        x_ddot = spl_dd[:, 0]
        y_ddot = spl_dd[:, 1]
        z_ddot = spl_dd[:, 2]

        # x,y path angle
        phi = np.arctan2(x_dot, y_dot)

        # derivative of the x,y path angle
        phi_dot = (np.multiply(x_dot, y_ddot) - np.multiply(y_dot, x_ddot)) / (np.square(x_dot) + np.square(y_dot))

        #### CONSTRAINTS ####

        # euclidean norm of the velocity
        v = np.sqrt(np.square(x_dot) + np.square(y_dot) + np.square(z_dot))

        # calculate flight path angle
        flight_path_angle = np.pi / 2 - np.arctan2(np.sqrt(np.square(x_dot) + np.square(y_dot)), z_dot)

        # calculate the roll angle
        roll_angle = np.arctan2(np.multiply(v, phi_dot), g)
        print(np.rad2deg(roll_angle))

        # calculate the thrust
        thrust = (np.cos(flight_path_angle) * np.cos(phi) * y_ddot + np.cos(flight_path_angle) * np.sin(
            phi) * x_ddot) * mass + F_drag

        fig, axs = plt.subplots(4)
        axs[0].plot(t,v,c='b')
        axs[0].plot(t, v_lower_limit*np.ones(len(t)),c='r')
        axs[0].plot(t, v_upper_limit * np.ones(len(t)),c='r')
        axs[0].title.set_text("Velocity")
        axs[1].plot(t, thrust,c='b')
        axs[1].plot(t, thrust_lower_limit*np.ones(len(t)),c='r')
        axs[1].plot(t, thrust_upper_limit * np.ones(len(t)),c='r')
        axs[1].title.set_text("Thrust")
        axs[2].plot(t, np.rad2deg(roll_angle), c='b')
        axs[2].plot(t, np.rad2deg(roll_angle_lower_limit*np.ones(len(t))),c='r')
        axs[2].plot(t, np.rad2deg(roll_angle_upper_limit * np.ones(len(t))),c='r')
        axs[2].title.set_text("Roll Angle")
        axs[3].plot(t, np.rad2deg(flight_path_angle),c='b')
        axs[3].plot(t, np.rad2deg(min_flight_path_angle*np.ones(len(t))),c='r')
        axs[3].plot(t, np.rad2deg(max_flight_path_angle * np.ones(len(t))),c='r')
        axs[3].title.set_text("flight path angle")
        plt.tight_layout()
        # plt.suptitle("spline "+str(i))
        plt.show(block = False)

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

    np.random.seed(20)


    #order of the spline
    k = 3

    #Starting and ending points of the trajectory
    #first vehicle
    pf_1 = np.array([0,0,0])
    p0_1 = np.array([0,0,20])

    #second vehicle
    pf_2 = np.array([0,0,0])
    p0_2 = np.array([0,0,20])

    #third vehicle
    pf_3 = np.array([0,0,0])
    p0_3 = np.array([0,0,20])

    p0 = [p0_1,p0_2,p0_3]
    pf = [pf_1,pf_2,pf_3]

    #desired starting and ending velocities of the trajectory
    #first vehicle
    v0_1 = np.array([0,-2,0])
    vf_1 =np.array([0,-5,-.1])

    #second vehicle
    v0_2 = np.array([0,-2,0])
    vf_2 =np.array([0,-5,-.1])

    #third vehicle
    v0_3 = np.array([0,-2,0])
    vf_3 =np.array([0,-5,-.1])

    v0 = [v0_1,v0_2,v0_3]
    vf = [vf_1,vf_2,vf_3]



    #number of control points for the spline
    num_cont_points = 14

    #upper and lower bound for control point values
    lb = -100
    ub = 100

    #velocity upper and lower bounds
    v_lower_limit = 1
    v_upper_limit = 20

    #thrust lower and upper bound
    thrust_lower_limit = -17
    thrust_upper_limit = 35

    #roll angle upper and lower bounds
    roll_angle_lower_limit = -np.deg2rad(25)
    roll_angle_upper_limit = np.deg2rad(25)

    #flight path angle lower and upper bounds
    min_flight_path_angle = -np.deg2rad(20)
    max_flight_path_angle = np.deg2rad(20)

    #starting time of the spine and initial condition for ending time
    t0 = 0
    tf_initial = 1

    #number of discrete sampels to break the spline into
    num_samples = 20

    #list of obstacles
    num_obst = 1
    x_lim= [5,10]
    y_lim = [5,10]
    z_lim = [3,6]
    min_radius = 4
    max_radius = 5
    obstacles = []#get_random_spherical_obsticales(num_obst, x_lim, y_lim, z_lim, min_radius, max_radius)
    print(obstacles)

    g = 9.8
    mass = 10.0
    F_drag = .268



    splines = min_func_spline_trajectory(p0, pf,v0,vf,k, num_cont_points, lb, ub, v_lower_limit,
                              v_upper_limit,thrust_lower_limit,thrust_upper_limit,roll_angle_lower_limit,roll_angle_upper_limit,t0,tf_initial,num_samples,
                               obstacles,min_flight_path_angle,max_flight_path_angle,g,mass,F_drag)

    ax = plt.axes(projection='3d')
    plot_splines(splines,100,ax)
    plot_spherical_obsticles(obstacles,ax)
    num_samples = 100
    plot_constraints(splines,v_lower_limit,v_upper_limit,thrust_lower_limit,thrust_upper_limit,roll_angle_lower_limit,roll_angle_upper_limit,min_flight_path_angle,max_flight_path_angle,num_samples,g,mass,F_drag)
    plt.show()
    # print(get_spline_dist(sp, num_samples))

if __name__ == "__main__":

    main()