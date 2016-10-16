#include <boost/bind.hpp>
#include <ros/ros.h>

#include <gazebo_msgs/ModelState.h>
#include <ros/subscribe_options.h>
#include <ros/callback_queue.h>

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>


#include <stdio.h>

namespace gazebo
{
  class ModelPush : public ModelPlugin
  {
    // quadcopter parameters

    // Posicion de los rotores
    // distancia: 25cms, inclinacion: 45º
    math::Vector3 pos_CM = math::Vector3( 0,0,0);  // centro de masas
    math::Vector3 pos_NE = math::Vector3( 0.1768,-0.1768, 0);
    math::Vector3 pos_NW = math::Vector3( 0.1768, 0.1768, 0);
    math::Vector3 pos_SE = math::Vector3(-0.1768,-0.1768, 0);
    math::Vector3 pos_SW = math::Vector3(-0.1768, 0.1768, 0);

    // Margen de velocidad de los motores
    private: const double w_max = 1.5708e+03; // rad/s = 15000rpm
    private: const double w_min = 0;          // rad/s =     0rpm

    
    /* Fuerza de empuje aerodinamico
       La fuerza principal generada por los rotores
       FT = kFT * w²  
       Asumimos que 
          FT_max = 1kg = 9.8N
       por tanto, queda que... */
    private: const double kFT = 3.9718e-06; 
    
    /* Momento de arrastre de los rotores
       Momento que experimenta el rotor en sentido contrario a su velocidad
       MDR = kMDR * w²
       Asumimos que 
          ...
       por tanto, queda que... */
    private: const double kMDR = 1.3581e-07; 

    /* Fuerza de arrastre aerodinamico.
       Fuerza de rozamiento con el aire, contraria a la velocidad.
       FD = -kFD * r_dot*|r_dot|  
       Depende de la forma del objeto en cada eje.  */
       
    /* Ejes horizontales:
       Asumimos 
          rozamiento similar en ambos ejes (aunque el fuselaje no sea igual)
          Vh_max = 20km/h = 5.5556m/s  (velocidad horizontal maxima)
          roll_max = 30º = 0.5236rad   (inclinacion maxima)
       operando
          FTh_max = 4*FT_max * sin(roll_max)
          FTh_max = FDh_max = 19.6
       por tanto queda que...  */
    private: const double kFDx = 0.6350;
    private: const double kFDy = 0.6350;
       
    /* Eje vertical:
       Debe verificarse a velocidad limite ascendente que
          FTmax * 4 = Fg + FD_max
       Asumimos que 
          Vz_max = 3m/s  (maxima velocidad de ascenso)
       que nos dará una velocidad limite de descenso de 
          Vz_lim = 2.7689m/s
       operando
          FT_max = 9.8N
          Fg = 1.840gr * 9.8m/s
          FD_max = 21.1681N
       por tanto queda que...  */
    private: const double kFDz = 2.3520;


    /* Momento de arrastre aerodinamico.
       Momento de rozamiento con el aire que sufre el drone al girar.
       Es contrario a la velocidad angular.
       MD = -kMD * rpy_dot * |rpy_dot|  
       Depende de la forma del objeto en cada eje.  */

    /* Ejes horizontales:
       Asumimos 
          rozamiento similar en ambos ejes (aunque el fuselaje no sea igual)
          escenario sin gravedad
          el drone es propulsado por dos rotores del mismo lado a maxima velocidad
          la velocidad angular maxima que alcanza es  Vrp_max = 2 * 2*pi;
       operando
          kMDxy =  2 * FT_max * sin(deg2rad(45))^2 / Vrp_max^2
       por tanto queda que...  */
    private: const double kMDx = 0.0621;
    private: const double kMDy = 0.0621;

    /* Eje vertical:
       Debe verificarse a velocidad limite de rotación sobre el eje Z que
          ...
       Asumimos que 
          Vyaw_max = 4*pi rad/s  (maxima velocidad de rotacion en Z de 2rev/s)
          w_hov2                 (velocidad del rotor para que dos rotores mantengan la sustentacion)
       Ya teniamos que
          MDR = kMDR * w²
          MDz = kMDz * Vyaw²
       operando
          MDz  = MDR             (el rozamiento con el aire compensa el efecto de los rotores)
          kMDz = kMDR* (2 * w_hov2²) / Vyaw_max²
       por tanto queda que...  */
    private: const double kMDz = 0.0039;

    

    // Pointers to the model
    private: physics::ModelPtr model;
    private: physics::LinkPtr  link;
    
    // Pointer to the update event connection
    private: event::ConnectionPtr updateConnection;

    // ROS management
    private: ros::NodeHandle* rosnode_;

    // Odometry publisher
    private: ros::Publisher pub_;
    private: common::Time last_odom_publish_time;
    private: double odom_publish_rate = 50.0;  // updates per second

    // Control subscriber
    private: ros::Subscriber sub_;
    private: std::string topic_subscripted_ = "rotor_speeds";
    private: ros::CallbackQueue queue_;
    private: boost::thread callback_queue_thread_;
    private: double w_rotor_NE = 0.0;  
    private: double w_rotor_NW = 0.0;  
    private: double w_rotor_SE = 0.0;  
    private: double w_rotor_SW = 0.0;  
  
    
    ////////////////////////////////////////////////////////
    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
    {
//      _sdf->PrintValues("\n\n\n");

      // Store pointers to the model
      this->model = _parent;
      this->link  = model->GetLink("dronelink");  
      
      // Listen to the update event. This event is broadcast every iteration.
      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          boost::bind(&ModelPush::OnUpdate, this, _1));

      // Ensure that ROS has been initialized
      if (!ros::isInitialized()) 
      {
        ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized.");
        return;
      }
  
      // Configure a ROS node 
      this->rosnode_ = new ros::NodeHandle("quadcopter");
          
        
      // Initiates the publication topic
      this->pub_ = this->rosnode_->advertise<gazebo_msgs::ModelState>("odometry",10);
      last_odom_publish_time = model->GetWorld()->GetSimTime();
      
      
      
      // Initiates the subcripted topic

      ros::SubscribeOptions so = ros::SubscribeOptions::create<geometry_msgs::Quaternion>(
              topic_subscripted_,
              1000,
              boost::bind(&ModelPush::rotorSpeedsCallBack, this, _1),
              ros::VoidPtr(), &this->queue_
              );
      this->sub_ = this->rosnode_->subscribe(so);
      this->callback_queue_thread_ = 
              boost::thread( boost::bind( &ModelPush::QueueThread,this ) );
      
    }

    public: void rotorSpeedsCallBack(const geometry_msgs::Quaternion::ConstPtr& msg)
    {
        
        /*
        rostopic pub -1 
                 /quadcopter/rotor_speeds        
                 geometry_msgs/Quaternion
                 -- '1' '02 '03 '4'
        */
        
//      printf("w_ctrl_NE: %.2f \nw_ctrl_NW: %.2f \nw_ctrl_SE: %.2f \nw_ctrl_SW: %.2f \n---\n",
//                msg->x,msg->y,msg->z,msg->w);

        w_rotor_NE = msg->x;  
        w_rotor_NW = msg->y;  
        w_rotor_SE = msg->z;  
        w_rotor_SW = msg->w;  

/*
        // Filtramos rotaciones negativas
        if(w_rotor_NE < 0) w_rotor_NE = 0;
        if(w_rotor_NW < 0) w_rotor_NW = 0;
        if(w_rotor_SE < 0) w_rotor_SE = 0;
        if(w_rotor_SW < 0) w_rotor_SW = 0;
        
        // Aplicamos saturacion de motores
        if(w_rotor_NE > w_max) w_rotor_NE = w_max;
        if(w_rotor_NW > w_max) w_rotor_NW = w_max;
        if(w_rotor_SE > w_max) w_rotor_SE = w_max;
        if(w_rotor_SW > w_max) w_rotor_SW = w_max;
*/
//      printf("w_rotor_NE: %.2f \nw_rotor_NW: %.2f \nw_rotor_SE: %.2f \nw_rotor_SW: %.2f \n---\n",
//                w_rotor_NE,w_rotor_NW,w_rotor_SE,w_rotor_SW);


    }

    ////////////////////////////////////////////////////////
    // Called by the world update start event
    public: void OnUpdate(const common::UpdateInfo & /*_info*/)
    {

      // Getting model status
      math::Pose pose = model->GetWorldPose();
//    printf("drone xyz = %.2f,%.2f,%.2f \n", pose.pos.x, pose.pos.y, pose.pos.z);
//    printf("drone quaternion = %.2f,%.2f,%.2f,%.2f \n", 
//      pose.rot.x, pose.rot.y, pose.rot.z, pose.rot.w);
      math::Vector3 linear_vel = model->GetRelativeLinearVel();
//    printf("drone vel xyz = %.2f,%.2f,%.2f \n", linear_vel.x, linear_vel.y, linear_vel.z);
      math::Vector3 angular_vel = model->GetRelativeAngularVel();
//    printf("drone angular vel xyz = %.2f,%.2f,%.2f \n", angular_vel.x, angular_vel.y, angular_vel.z);



      // Read rotor speeds from topic


/*
      // con esto simulamos rotacion de sustentacion
      w_rotor_NE = sqrt(1.840*9.8/4 / kFT);  // = 1065.4  con 4 rotores
      w_rotor_NW = omega_NE;
      w_rotor_SE = omega_NE;
      w_rotor_SW = omega_NE;
*/

/*
      // con esto simulamos rotacion de sustentacion con solo dos rotores
      // aplicamos maximo giro en vertical
      w_rotor_NE = sqrt(1.840*9.8/2 / kFT);  // = 1506.7
      w_rotor_NW = 0;
      w_rotor_SE = 0;
      w_rotor_SW = omega_NE;
*/    



    
      // aplicamos fuerzas/momentos por empuje de rotores
      math::Vector3 FT_NE = math::Vector3(0, 0, kFT * pow(w_rotor_NE,2));
      link->AddLinkForce(FT_NE,pos_NE);
      math::Vector3 FT_NW = math::Vector3(0, 0, kFT * pow(w_rotor_NW,2));
      link->AddLinkForce(FT_NW,pos_NW);
      math::Vector3 FT_SE = math::Vector3(0, 0, kFT * pow(w_rotor_SE,2));
      link->AddLinkForce(FT_SE,pos_SE);
      math::Vector3 FT_SW = math::Vector3(0, 0, kFT * pow(w_rotor_SW,2));
      link->AddLinkForce(FT_SW,pos_SW);
      
      // aplicamos momentos por arrastre de rotores
      math::Vector3 MDR_NE = math::Vector3(0, 0, kMDR * pow(w_rotor_NE,2));
      math::Vector3 MDR_NW = math::Vector3(0, 0, kMDR * pow(w_rotor_NW,2));
      math::Vector3 MDR_SE = math::Vector3(0, 0, kMDR * pow(w_rotor_SE,2));
      math::Vector3 MDR_SW = math::Vector3(0, 0, kMDR * pow(w_rotor_SW,2));
//    printf("MDR  = %.15f\n",MDR_NE.z - MDR_NW.z - MDR_SE.z + MDR_SW.z); 
      link->AddRelativeTorque(MDR_NE - MDR_NW - MDR_SE + MDR_SW);
      

      // aplicamos fuerza de rozamiento con el aire
      math::Vector3 FD = math::Vector3(
              -kFDx * linear_vel.x * fabs(linear_vel.x),
              -kFDy * linear_vel.y * fabs(linear_vel.y),
              -kFDz * linear_vel.z * fabs(linear_vel.z)
              );
//    printf("drone relative vel \nbZ  %.2f\n|Z| %.2f\nFDz %.2f \n\n",
//      linear_vel.z, fabs(linear_vel.z), -kFDz * linear_vel.z * fabs(linear_vel.z) );
      link->AddLinkForce(FD,pos_CM);
      
      // aplicamos momento de rozamiento con el aire
      math::Vector3 MD = math::Vector3(
              -kMDx * angular_vel.x * fabs(angular_vel.x),
              -kMDy * angular_vel.y * fabs(angular_vel.y),
              -kMDz * angular_vel.z * fabs(angular_vel.z)
              );
      link->AddRelativeTorque(MD);
      



      // Is it time to publish odometry to topic?
      common::Time current_time = model->GetWorld()->GetSimTime();
      if (current_time < last_odom_publish_time)
        last_odom_publish_time = current_time;    // The simulation was reset
//      printf("current time:  %.3f \n", current_time.Double());
//      printf("last time:     %.3f \n", last_odom_publish_time.Double());
      double seconds_since_last_update = (current_time - last_odom_publish_time).Double();
      if (seconds_since_last_update > (1.0 / odom_publish_rate)) 
      {
        // Publish odometry to topic
//      printf("Publishing odometry--------------\n");

        gazebo_msgs::ModelState msg;
        msg.model_name = "quadcopter";

        msg.pose.position.x = pose.pos.x;
        msg.pose.position.y = pose.pos.y;
        msg.pose.position.z = pose.pos.z;

        msg.pose.orientation.x = pose.rot.x;
        msg.pose.orientation.y = pose.rot.y;
        msg.pose.orientation.z = pose.rot.z;
        msg.pose.orientation.w = pose.rot.w;

        msg.twist.linear.x = linear_vel.x;
        msg.twist.linear.y = linear_vel.y;
        msg.twist.linear.z = linear_vel.z;

        msg.twist.angular.x = angular_vel.x;
        msg.twist.angular.y = angular_vel.y;
        msg.twist.angular.z = angular_vel.z;

        pub_.publish(msg);

        last_odom_publish_time = current_time;
      }    


    
    }
    
    // Custom Callback Queue
    ////////////////////////////////////////////////////////////////////////////////
    // custom callback queue thread
    void QueueThread()
    {
      static const double timeout = 0.01;

      while (this->rosnode_->ok())
      {
        this->queue_.callAvailable(ros::WallDuration(timeout));
      }
    }
    
    
    
  };






  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(ModelPush)
}
