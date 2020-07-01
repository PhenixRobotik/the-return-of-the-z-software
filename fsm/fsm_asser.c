#include "fsm_asser.h"

void init_FSM_asser(FSM_asser *fsm_asser,PID_Status *pid_sigma,PID_Status *pid_theta)//init the FSM in a stop state
{
  FSM_Instance *fsm=&(fsm_asser->instance);
  fsm->status=FSM_SUCCESS;

  fsm_asser->pid_sigma=pid_sigma;
  fsm_asser->pid_theta=pid_theta;

  set_stop(fsm_asser);
}

void set_stop(FSM_asser *fsm_asser)//stop the robot at the current position and the fsm
{
  odometry odom=odometry_get_position();
  fsm_asser->sum_goal=0.5 * (odom.left_total_distance + odom.right_total_distance);//stop at current position
  fsm_asser->diff_goal=odom.right_total_distance - odom.left_total_distance;

  fsm_asser->fsm_pointer=0;//nothing to schedule
  fsm_asser->fsm_pointer_max=0;

  FSM_Instance *fsm=&(fsm_asser->instance);//stop the fsm execution
  FSM_NEXT(fsm,FSM_asser_wait_end,0);
}




void set_translation_speed(FSM_asser *fsm_asser,double vt)//in mm/s
{
  fsm_asser->linear_speed=vt;
}

void set_translation(FSM_asser *fsm_asser,double t)
{
  fsm_asser->pos=t;
  fsm_asser->initial_sum=fsm_asser->sum_goal;
  set_trapezoid(fsm_asser,t,fsm_asser->linear_speed);
  FSM_Instance *fsm=&(fsm_asser->instance);
  FSM_NEXT(fsm,FSM_asser_translation,0);
  fsm->status=FSM_RUNNING;

  fsm_asser->fsm_pointer=0;//nothing to schedule
  fsm_asser->fsm_pointer_max=0;
}

void set_theta_speed(FSM_asser *fsm_asser,double vth)
{
  fsm_asser->angular_speed=vth;
}

void set_theta(FSM_asser *fsm_asser,double theta)//TODO: set the shortest angle, attention theta0
{
  fsm_asser->angle=theta;
  fsm_asser->initial_diff=fsm_asser->diff_goal;
  set_trapezoid(fsm_asser,theta,fsm_asser->angular_speed);
  FSM_Instance *fsm=&(fsm_asser->instance);
  FSM_NEXT(fsm,FSM_asser_angle,0);
  fsm->status=FSM_RUNNING;

  fsm_asser->fsm_pointer=0;//nothing to schedule
  fsm_asser->fsm_pointer_max=0;
}



void set_X_Y_theta(FSM_asser *fsm_asser,double x,double y,double theta,int back)
{
  fsm_asser->X_goal=x;
  fsm_asser->Y_goal=y;
  fsm_asser->theta_goal=theta;
  fsm_asser->back=back;

  fsm_asser->fsm_scheduler[0]=set_X_Y_theta_translation;
  fsm_asser->fsm_scheduler[1]=set_X_Y_theta_rotation;
  fsm_asser->fsm_pointer=0;
  fsm_asser->fsm_pointer_max=2;

  odometry odom=odometry_get_position();
  double trajectory_angle=atan2(y-odom.y,x-odom.x);
  if(back==1)
  {
    trajectory_angle+=Pi;
  }
  fsm_asser->angle=trajectory_angle-odom.theta;
  fsm_asser->angle=limit_angle(fsm_asser->angle);


  fsm_asser->initial_diff=fsm_asser->diff_goal;
  set_trapezoid(fsm_asser,fsm_asser->angle,fsm_asser->angular_speed);
  FSM_Instance *fsm=&(fsm_asser->instance);
  FSM_NEXT(fsm,FSM_asser_angle,0);
  fsm->status=FSM_RUNNING;

}

void set_X_Y_theta_translation(FSM_asser *fsm_asser)
{
  odometry odom=odometry_get_position();

  double dx=fsm_asser->X_goal-odom.x;
  double dy=fsm_asser->Y_goal-odom.y;
  double t=sqrt(dx*dx+dy*dy);
  if(fsm_asser->back==1)
  {
    t*=-1;
  }

  fsm_asser->pos=t;
  fsm_asser->initial_sum=fsm_asser->sum_goal;
  set_trapezoid(fsm_asser,t,fsm_asser->linear_speed);
  FSM_Instance *fsm=&(fsm_asser->instance);
  FSM_NEXT(fsm,FSM_asser_translation,0);
  fsm->status=FSM_RUNNING;
}

void set_X_Y_theta_rotation(FSM_asser *fsm_asser)
{
  odometry odom=odometry_get_position();
  fsm_asser->angle=fsm_asser->theta_goal-odom.theta;
  fsm_asser->angle=limit_angle(fsm_asser->angle);

  fsm_asser->initial_diff=fsm_asser->diff_goal;
  set_trapezoid(fsm_asser,fsm_asser->angle,fsm_asser->angular_speed);

  FSM_Instance *fsm=&(fsm_asser->instance);
  FSM_NEXT(fsm,FSM_asser_angle,0);
  fsm->status=FSM_RUNNING;
}

void set_trapezoid(FSM_asser *fsm_asser,double goal, double speed)
{
  fsm_asser->goal=goal;
  speed=fabs(speed);
  if(goal<0.0)
  {
    speed *= -1;
  }
  fsm_asser->speed=speed;

  fsm_asser->t0 = SYSTICK_TO_MILLIS(get_systicks())/1000.0;
  fsm_asser->t3 = goal/(speed*(1-0.5*X1-0.5*X2));//total time of ramp generation
  fsm_asser->t1 = X1*fsm_asser->t3;
  fsm_asser->t2 = (1-X2)*fsm_asser->t3;

  fsm_asser->a1 = speed/fsm_asser->t1;
  fsm_asser->a2 = speed/(fsm_asser->t3-fsm_asser->t2);

  fsm_asser->x1 = 0.5*fsm_asser->a1*fsm_asser->t1*fsm_asser->t1;
  fsm_asser->x2 = speed*(fsm_asser->t2-fsm_asser->t1)+fsm_asser->x1;
}

double get_trapezoid(FSM_asser *fsm_asser)
{
  double t = (SYSTICK_TO_MILLIS(get_systicks())/1000.0)-fsm_asser->t0,dt;

  if(t<fsm_asser->t1)
  {
    return 0.5*fsm_asser->a1*t*t;
  }
  else if(t<fsm_asser->t2)
  {
    return fsm_asser->speed*(t-fsm_asser->t1)+fsm_asser->x1;
  }
  else if (t<=fsm_asser->t3)
  {
    dt=t-fsm_asser->t2;
    return -0.5*fsm_asser->a2*dt*dt+fsm_asser->speed*dt+fsm_asser->x2;
  }
  return fsm_asser->goal;
}

void get_order(FSM_asser *fsm_asser,double *sum_goal,double *diff_goal)//just return the orders
{
  *sum_goal=fsm_asser->sum_goal;
  *diff_goal=fsm_asser->diff_goal;
}



void FSM_asser_translation(FSM_Instance *fsm)
{
  FSM_asser *fsm_asser=(FSM_asser *) fsm;

  double pos=get_trapezoid(fsm_asser);

  if(fabs(pos)>=fabs(fsm_asser->pos))
  {
    pos=fsm_asser->pos;
    FSM_NEXT(fsm,FSM_asser_wait_end,0);//done
  }

  fsm_asser->sum_goal=pos+fsm_asser->initial_sum;
}


void FSM_asser_angle(FSM_Instance *fsm)
{
  FSM_asser *fsm_asser=(FSM_asser *) fsm;

  double angle = get_trapezoid(fsm_asser);

  if(fabs(angle)>=fabs(fsm_asser->angle))
  {
    angle=fsm_asser->angle;
    FSM_NEXT(fsm,FSM_asser_wait_end,0);//done
  }
  fsm_asser->diff_goal=angle*Encoders_Axis_Distance+fsm_asser->initial_diff;
}

void FSM_asser_wait_end(FSM_Instance *fsm)
{
  FSM_asser *fsm_asser=(FSM_asser *) fsm;
  if(reached(fsm_asser->pid_theta) && reached(fsm_asser->pid_sigma))
  {
    if(fsm_asser->fsm_pointer>=fsm_asser->fsm_pointer_max)
    {
      FSM_NEXT(fsm,FSM_NOP,0);
    }
    else
    {
      void (*to_run)(FSM_asser *fsm_asser)=(fsm_asser->fsm_scheduler)[fsm_asser->fsm_pointer];
      to_run(fsm_asser);//runs the next setup function
      fsm_asser->fsm_pointer++;
    }
  }
}

double limit_angle(double angle)
{
  while(angle >Pi)
  {
    angle-=2*Pi;
  }
  while(angle <=-Pi)
  {
    angle+=2*Pi;
  }
  return angle;
}
