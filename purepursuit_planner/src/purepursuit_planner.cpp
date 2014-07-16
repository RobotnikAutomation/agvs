/** \file purepursuit_planner.cc
 * \author Robotnik Automation S.L.L.
 * \version 1.0
 * \date    2014
 *
 * \brief purepursuit_planner class motorsDev
 * Component to manage the Agvs controller
 * (C) 2014 Robotnik Automation, SLL
*/
#include <string.h>
#include <vector>
#include <queue>
#include <stdint.h>
#include <ros/ros.h>
#include <math.h>
#include <cstdlib>

#include <purepursuit_planner/Component.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include "diagnostic_msgs/DiagnosticStatus.h"
#include "diagnostic_updater/diagnostic_updater.h"
#include "diagnostic_updater/update_functions.h"
#include "diagnostic_updater/DiagnosticStatusWrapper.h"
#include "diagnostic_updater/publisher.h"
#include <actionlib/server/simple_action_server.h>
#include <planner_msgs/GoToAction.h>
#include <planner_msgs/goal.h>
#include <geometry_msgs/Pose2D.h>
//#include <s3000_laser/enable_disable.h>


#define ODOM_TIMEOUT_ERROR			0.2				// max num. of seconds without receiving odom values
#define MAP_TIMEOUT_ERROR			0.2				// max num. of seconds without receiving map transformations
#define AGVS_TURN_RADIUS			0.20			// distancia en la que empieza a girar el robot cuando llega a una esquina
#define MIN_ANGLE_BEZIER			0.261799388		// ángulo (radianes) mínimo entre segmentos de la recta para los que ajustaremos a una curva de BEZIER
#define BEZIER_CONTROL_POINTS		5

#define D_LOOKAHEAD_MIN				0.3		// Minima distancia del punto objetivo en m (PurePursuit con lookahead dinámico)
#define D_LOOKAHEAD_MAX				1.1		// Maxima distancia del punto objetivo
#define D_WHEEL_ROBOT_CENTER    	0.478   // Distance from the motor wheel to the robot center

#define MAX_SPEED_LVL1				0.5
#define MAX_SPEED_LVL2				0.3
#define MAX_SPEED					1.2

#define WAYPOINT_POP_DISTANCE_M		0.10		//Distancia mínima para alcanzar punto objetivo m (PurePursuit)

#define AGVS_FIRST_DECELERATION_DISTANCE 	0.5 	// meters -> when the vehicle is arriving to the goal, it has to decelarate at this distance
#define AGVS_FIRST_DECELERATION_MAXSPEED	0.15	// m/s
#define AGVS_SECOND_DECELERATION_DISTANCE   0.25 	// meters -> when the vehicle is arriving to the goal, it has to decelarate another time at this distance
#define AGVS_SECOND_DECELERATION_MAXSPEED	0.1 	// m/s
#define AGVS_DEFAULT_KR	0.20				// 

enum{
	ODOM_SOURCE = 1,
	MAP_SOURCE = 2
};

using namespace std;

//! Data structure for a Magnet
typedef struct MagnetStruct{
    //! Id of the magnet
    int iID;
    //! X position
    double dX;
    //! Y position
    double dY;
}MagnetStruct;

//! Data structure for a Waypoint
typedef struct Waypoint{
    //! Id of the magnet
    int iID;
    //! X position
    double dX;
    //! Y position
    double dY;
    //! Orientation
    double dA;
    //! Speed to arrive to the point
    double dSpeed;
}Waypoint;

//! class to manage the waypoints and magnets of the current path
class Path{
	public:
	//! Current waypoint
	int iCurrentWaypoint;
	//! Current magnet
	int iCurrentMagnet;

	private:
	//! Mutex to control the access
	pthread_mutex_t mutexPath;
	//! Vector to store all the Waypoints
	vector <Waypoint> vPoints;
	//! Vector to store all the magnets
	vector <MagnetStruct> vMagnets;
	//! Flag to control the optimization
	bool bOptimized;

	public:

	//! public constructor
	Path(){
		iCurrentWaypoint = iCurrentMagnet = -1;
		pthread_mutex_init(&mutexPath, NULL);//Initialization for WaypointRoutes' mutex
		bOptimized = false;
	}

	//! Destructor
	~Path(){
		pthread_mutex_destroy(&mutexPath);
	}

	//! Adds a new waypoint
	ReturnValue AddWaypoint(Waypoint point){
		Waypoint aux;

		pthread_mutex_lock(&mutexPath);
			if(vPoints.size() > 0){
				aux = vPoints.back();
				// Only adds the waypoint if it's different from waypoint before
				if( (aux.dX != point.dX) || (aux.dY != point.dY) )
					vPoints.push_back(point);
			} else { // First point
				if(iCurrentWaypoint < 0){ //First point
					iCurrentWaypoint = 0;
				}

				vPoints.push_back(point);
			}

		pthread_mutex_unlock(&mutexPath);

		return OK;
	}

	//! Adds a vector of waypoints
	ReturnValue AddWaypoint(vector <Waypoint> po){
		pthread_mutex_lock(&mutexPath);
			if(iCurrentWaypoint < 0){ //First point
				iCurrentWaypoint = 0;
			}
			for(int i = 0; i < po.size(); i++){
				vPoints.push_back(po[i]);
			}
		pthread_mutex_unlock(&mutexPath);
	}

	//! Adds a new magnet
	ReturnValue AddMagnet(MagnetStruct magnet){
		pthread_mutex_lock(&mutexPath);
			if(iCurrentMagnet < 0){ //First point
				iCurrentMagnet = 0;
			}
			vMagnets.push_back(magnet);
		pthread_mutex_unlock(&mutexPath);

		return OK;
	}

	//! Adds a vector of magnets
	ReturnValue AddMagnet(vector <MagnetStruct> po){
		pthread_mutex_lock(&mutexPath);
			if(iCurrentMagnet < 0){ //First point
				iCurrentMagnet = 0;
			}
			for(int i = 0; i < po.size(); i++){
				vMagnets.push_back(po[i]);
			}
		pthread_mutex_unlock(&mutexPath);
	}

	//! Clears the waypoints and magnets
	void Clear(){
		pthread_mutex_lock(&mutexPath);
			iCurrentWaypoint = -1;
			iCurrentMagnet = -1;
			bOptimized = false;
			vPoints.clear();
			vMagnets.clear();
		pthread_mutex_unlock(&mutexPath);
	}
	
	//! Returns the size of the vector points
	unsigned int Size(){
		return vPoints.size();
	}

	//! Returns the next waypoint
	ReturnValue GetNextWaypoint(Waypoint *wp){
		ReturnValue ret = ERROR;

		pthread_mutex_lock(&mutexPath);

			if( (iCurrentWaypoint >= 0) && (iCurrentWaypoint < (vPoints.size() - 1)) ){
				*wp = vPoints[iCurrentWaypoint + 1];
				ret = OK;
			}

		pthread_mutex_unlock(&mutexPath);

		return ret;
	}

	//! Returns the last waypoint
	ReturnValue BackWaypoint(Waypoint *wp){
		ReturnValue ret = ERROR;

		pthread_mutex_lock(&mutexPath);
			if( vPoints.size() > 0){
				*wp = vPoints.back();
				ret = OK;
			}
		pthread_mutex_unlock(&mutexPath);

		return ret;
	}

	//! Gets the current waypoint
	ReturnValue GetCurrentWaypoint(Waypoint *wp){
		ReturnValue ret = ERROR;

		pthread_mutex_lock(&mutexPath);
			if( (iCurrentWaypoint >= 0) && (iCurrentWaypoint < vPoints.size()) ){
				*wp = vPoints[iCurrentWaypoint];
				ret = OK;
			}
		pthread_mutex_unlock(&mutexPath);

		return ret;
	}

	//! Gets selected waypoint
	ReturnValue GetWaypoint(int index, Waypoint *wp){
		ReturnValue ret = ERROR;

		pthread_mutex_lock(&mutexPath);
			if( (index >= 0) && ( index< vPoints.size() ) ){
				*wp = vPoints[index];
				ret = OK;
			}
		pthread_mutex_unlock(&mutexPath);

		return ret;
	}

	//! Gets the current Waypoint in the path
	int GetCurrentWaypointIndex(){
		return iCurrentWaypoint;
	}

	//!	Sets the current Waypoint to index
	ReturnValue SetCurrentWaypoint(int index){
		ReturnValue ret = ERROR;

		if(index < (vPoints.size() - 1)){
			pthread_mutex_lock(&mutexPath);
				iCurrentWaypoint = index;
			pthread_mutex_unlock(&mutexPath);
			ret = OK;
		}

		return ret;
	}

	 //! Increase waypoint's number
	void NextWaypoint(){
		pthread_mutex_lock(&mutexPath);
			iCurrentWaypoint++;
		pthread_mutex_unlock(&mutexPath);
	}

	//! Returns the number of waypoints
	int NumOfWaypoints(){
		return vPoints.size();
	}

	//! Returns the next magnet
	ReturnValue GetNextMagnet(MagnetStruct *mg){
		ReturnValue ret = ERROR;
		pthread_mutex_lock(&mutexPath);

			if( (iCurrentMagnet >= 0) && (iCurrentMagnet < (vMagnets.size() - 1)) ){
				*mg = vMagnets[iCurrentMagnet + 1];
				ret = OK;
			}

		pthread_mutex_unlock(&mutexPath);


		return ret;
	}

	//! Returns the back magnet
	ReturnValue BackMagnet(MagnetStruct * mg){
		ReturnValue ret = ERROR;

		 pthread_mutex_lock(&mutexPath);
			if( vMagnets.size() > 0){
				*mg = vMagnets.back();
				ret = OK;
			}
		pthread_mutex_unlock(&mutexPath);

		return ret;
	}

	//! Gets the current magnet
	ReturnValue GetCurrentMagnet( MagnetStruct * mg ){
		ReturnValue ret = ERROR;

		pthread_mutex_lock(&mutexPath);
			if( (iCurrentMagnet >= 0) && (iCurrentMagnet < vMagnets.size()) ){
				*mg = vMagnets[iCurrentMagnet];
				ret = OK;
			}
		pthread_mutex_unlock(&mutexPath);

		return ret;
	}

	//! Gets the previous magnet
	ReturnValue GetPreviousMagnet( MagnetStruct * mg ){
		ReturnValue ret = ERROR;

		pthread_mutex_lock(&mutexPath);
			if( (iCurrentMagnet > 0) && (iCurrentMagnet <= vMagnets.size()) ){
				*mg = vMagnets[iCurrentMagnet-1];
				ret = OK;
			}
		pthread_mutex_unlock(&mutexPath);

		return ret;
	}

	//! Gets the current MagnetStruct in the path
	int GetCurrentMagnetIndex(){
		return iCurrentMagnet;
	}

	//!	Sets the current magnet to index
	ReturnValue SetCurrentMagnet(int index){
		ReturnValue ret = ERROR;

		if(index < (vMagnets.size() - 1)){
			pthread_mutex_lock(&mutexPath);
				iCurrentMagnet = index;
			pthread_mutex_unlock(&mutexPath);
			ret = OK;
		}
		return ret;
	}

	//! Increase magnet's number
	void NextMagnet(){
		pthread_mutex_lock(&mutexPath);
			iCurrentMagnet++;
		pthread_mutex_unlock(&mutexPath);
	}

	 //! Gets the last magnet
	ReturnValue GetLastMagnet( MagnetStruct * mg ){
		ReturnValue ret = ERROR;

		pthread_mutex_lock(&mutexPath);
			if( (iCurrentMagnet > 0) && (iCurrentMagnet <= vMagnets.size() ) ){
				*mg = vMagnets[iCurrentMagnet - 1];
				ret = OK;
			}
		pthread_mutex_unlock(&mutexPath);

		return ret;
	}

	  //! Returns the number of magnets
	int NumOfMagnets(){
		return vMagnets.size();
	}

	//! Overloaded operator +=
	Path &operator+=(const Path &a){
		AddWaypoint(a.vPoints);
		AddMagnet(a.vMagnets);
		return *this;
	}

	//! Cross product
	double dot2( Waypoint w, Waypoint v) {
		return (w.dX*v.dX + w.dY*v.dY);
	}

	//! Obtains the points for a quadratic Bezier's curve
	//! \param cp0 as player_pose2d_t, control point 0
	//!	\param cp1 as player_pose2d_t, control point 1
	//!	\param cp2 as player_pose2d_t, control point 2
	//!	\param t as float, [0 ... 1]
	//!	\return Point over the curve
	Waypoint PosOnQuadraticBezier(Waypoint cp0, Waypoint cp1, Waypoint cp2, float t){
		Waypoint aux;

		//B(t)= (1-t)^2 * cp0 + 2*t*(1-t)*cp1 + t^2 * cp2;
		//Bx(t)
		aux.dX = (1.0-t)*(1.0-t)*cp0.dX + 2*t*(1.0-t)*cp1.dX + t*t*cp2.dX;
		//By(t)
		aux.dY = (1.0-t)*(1.0-t)*cp0.dY + 2*t*(1.0-t)*cp1.dY + t*t*cp2.dY;

		return aux;
	}

	//! Function that calculate the distance to deccelerate from target speed
	//! \param target_speed as double, speed on m/s
	//! \return distance on meters
	double DistForSpeed(double target_speed){
		if(target_speed > 1.0)
			return 2.0;
		else if(target_speed > 0.8)
			return 1.5;
		else return
			1.0;

	}

	//! Modifies and adds new waypoints to the route for improving the path
	//! \param distance as double, used for the calculation of the new points
	//! \return ERROR if Size is lower than 3, distance <= 0 or the waypoints has already been optimized
	//! \return OK
	ReturnValue Optimize(double distance){
		int i, j=0;
		int a, b, c;
		int x = 0, y = 1, speed = 2;
		double mod_ab, mod_bc;
		double dAngle;
		Waypoint ab, bc, ba;
		double K= 0.0;
		vector <Waypoint> new_points;
		Waypoint aux;
		double Ax = 0.0, Ay = 0.0, Bx = 0.0, By = 0.0, Cx = 0.0, Cy = 0.0;
		Waypoint A, B, C;
		double Kt = 1.0 / BEZIER_CONTROL_POINTS;
		double dAuxSpeed = 0.0;
		double dMinDist = 0.0;	// Minica distancia a la que hay q frenar en funcion de la velocidad

		if(bOptimized){	//Already optimized
			return OK;
		}

		if((vPoints.size() < 2) || (distance <= 0.0)){	//Minimo 3 puntos en la ruta
			//printf("WaypointRoute::Optimize: Error: not enought points (%d)\n",Size());
			return ERROR;
		}
		pthread_mutex_lock(&mutexPath);
			//
			// Si solo hay dos puntos, interpolamos y creamos un punto intermedio
			if(vPoints.size() == 2){
				aux = vPoints[1];
				vPoints.push_back(aux); // Añadimos un punto al final y modificamos el del medio
				if((vPoints[0].dX - aux.dX) == 0.0){// Punto en el mismo eje X
					vPoints[1].dX = vPoints[0].dX;    // Mantenemos la coordenada en X
					vPoints[1].dY = vPoints[0].dY + (aux.dY - vPoints[0].dY) / 2.0; // La coordenada en Y será igual a la del primer punto más la mitada de distancia entre el punto final e inicial
				}else if((vPoints[0].dY - aux.dY) == 0.0){ // Punto en el mismo eje Y
					vPoints[1].dX = vPoints[0].dX + (aux.dX - vPoints[0].dX) / 2.0; // La coordenada en X será igual a la del primer punto más la mitada de distancia entre el punto final e inicial
					vPoints[1].dY = vPoints[0].dY;    // Mantenemos la coordenada en Y
				}else{ // Punto en eje X, Y distinto
					vPoints[1].dX = vPoints[0].dX + (aux.dX - vPoints[0].dX) / 2.0; // La coordenada en X será igual a la del primer punto más la mitada de distancia entre el punto final e inicial
					vPoints[1].dY = vPoints[0].dY + (aux.dY - vPoints[0].dY) / 2.0; // La coordenada en Y será igual a la del primer punto más la mitada de distancia entre el punto final e inicial
				}
			}
		
			new_points.push_back(vPoints[0]);
			new_points.push_back(vPoints[1]);

			for(i=2; i < vPoints.size(); i++){	// Primera pasada, añadimos puntos para los giros en curvas
				//cout << " Partiendo de punto " << i << endl;
				a = i-2;
				b = i-1;
				c = i;

				Ax = vPoints[a].dX;
				Ay = vPoints[a].dY;
				Bx = vPoints[b].dX;
				By = vPoints[b].dY;
				Cx = vPoints[c].dX;
				Cy = vPoints[c].dY;

				ab.dX = Bx - Ax;
				ab.dY = By - Ay;
				mod_ab = sqrt(ab.dX * ab.dX + ab.dY * ab.dY);

				bc.dX = Cx - Bx;
				bc.dY = Cy - By;
				mod_bc = sqrt(bc.dX * bc.dX + bc.dY * bc.dY);

				dAngle= acos(dot2(ab,bc)/(mod_ab*mod_bc));
				//cout <<  i << " Angle =  "<< dAngle << endl;
				if(fabs(dAngle) >= MIN_ANGLE_BEZIER){ // en caso del angulo que forman los segmentos sea menor, entonces generaremos puntos para aproximar mjor la curva
					// siendo ba vector director de b->a y bc el vector director b->c
					// Calculamos un punto a una distancia 'd' desde 'b' hacia 'a' y otro punto desde 'b' hacia 'c'
					// Estos puntos serán los que se añadirán a la lista de waypoints para poder trazar una curva de bezier

					new_points.pop_back();

					// Calcula velocidad maxima en funcion del giro de la curva
					if(fabs(dAngle) >= (Pi/4)){
						dAuxSpeed = MAX_SPEED_LVL2;
					}else
						dAuxSpeed = MAX_SPEED_LVL1;
					//cout << "Aux speed = " << dAuxSpeed << ", Next speed =  " << vPoints[b].dSpeed << endl;
					// Si la velocidad en ese waypoint supera el máximo establecido para un giro así
					if(fabs(vPoints[b].dSpeed) > dAuxSpeed){

						if(vPoints[b].dSpeed < 0.0)	// Cambiamos sentido de avance
							dAuxSpeed = -dAuxSpeed;

						dMinDist = DistForSpeed(fabs(vPoints[b].dSpeed));
						//cout << "Min dist = " << dMinDist  << endl;
						// Si el punto antes del giro esta a una distancia menor a la mínima, añadimos nuevo punto a un metro
						if( mod_ab > dMinDist){
							//Lo creamos
							ba.dX = -ab.dX;
							ba.dY = -ab.dY;
							K = dMinDist / sqrt(ba.dX * ba.dX + ba.dY * ba.dY);

							aux.dX = Bx + K * ba.dX;	// x = x' + K*Vx
							aux.dY = By + K * ba.dY;	// y = y' + K*Vy	//(Vx, Vy) vector director
							aux.dSpeed = vPoints[b].dSpeed;
							//cout << "Nuevo punto en " << aux.dX << ", " << aux.dY << endl;
							new_points.push_back(aux);
						}

						vPoints[b].dSpeed = dAuxSpeed;

					}
					// El primer waypoint no se modifica
					if(mod_ab > distance){ // si la distancia entre ab es MAYOR a la distancia del punto que pretendemos crear, creamos un punto intermedio
						//Lo creamos
						ba.dX = -ab.dX;
						ba.dY = -ab.dY;
						K = distance / sqrt(ba.dX * ba.dX + ba.dY * ba.dY);

						aux.dX = Bx + K * ba.dX;	// x = x' + K*Vx
						aux.dY = By + K * ba.dY;	// y = y' + K*Vy	//(Vx, Vy) vector director
						aux.dSpeed = vPoints[b].dSpeed;
						//cout << "Nuevo punto en " << aux.dX << ", " << aux.dY << endl;
						new_points.push_back(aux);
						//j++;
					}

					new_points.push_back(vPoints[b]);

					if(mod_bc > distance){ // si la distancia entre ab es menor a la distancia del punto que pretendemos crear, lo dejamos como está
						//Lo creamos
						K = distance / sqrt(bc.dX * bc.dX + bc.dY * bc.dY);
						aux.dX = Bx + K * bc.dX;	// x = x' + K*Vx
						aux.dY = By + K * bc.dY;	// y = y' + K*Vy	//(Vx, Vy) vector director
						aux.dSpeed = vPoints[b].dSpeed;
						//j++;
						//cout << "Nuevo punto en " << aux.dX << ", " << aux.dY << endl;
						new_points.push_back(aux);
					}

					// Creamos punto para después del giro
					if(dMinDist > 0.0) {
						if(mod_bc > 1.0){	// si la distancia al punto C es mayor que 1 metro, después del giro
							// Creamos un nuevo punto
							//Lo creamos
							K = 1.0 / sqrt(bc.dX * bc.dX + bc.dY * bc.dY);
							aux.dX = Bx + K * bc.dX;	// x = x' + K*Vx
							aux.dY = By + K * bc.dY;	// y = y' + K*Vy	//(Vx, Vy) vector director
							aux.dSpeed = vPoints[b].dSpeed;
							new_points.push_back(aux);
						}else{
							// Si no, establecemos una velocidad máxima
							vPoints[c].dSpeed = vPoints[b].dSpeed;
						}
					}


					new_points.push_back(vPoints[c]);
					//j++;
				}else{	//Se queda como está

					new_points.push_back(vPoints[c]);

				}
			}

			// Borramos antiguos waypoints e insertamos los nuevos
			vPoints.clear();

			// BEZIER
			vPoints.push_back(new_points[0]);
			vPoints.push_back(new_points[1]);
			for(i=2; i < new_points.size(); i++){	// Segunda pasada, aproximamos los giros a curvas de Bezier
				a = i-2;
				b = i-1;
				c = i;

				Ax = new_points[a].dX;
				Ay = new_points[a].dY;
				Bx = new_points[b].dX;
				By = new_points[b].dY;
				Cx = new_points[c].dX;
				Cy = new_points[c].dY;

				ab.dX = Bx - Ax;
				ab.dY = By - Ay;
				mod_ab = sqrt(ab.dX * ab.dX + ab.dY * ab.dY);

				bc.dX = Cx - Bx;
				bc.dY = Cy - By;
				mod_bc = sqrt(bc.dX * bc.dX + bc.dY * bc.dY);

				dAngle= acos(dot2(ab,bc)/(mod_ab*mod_bc));

				if(fabs(dAngle) >= MIN_ANGLE_BEZIER){ // en caso del angulo que forman los segmentos sea menor, entonces generaremos puntos, siguiendo una curva de Bezier, para aproximar mjor la curva
					// siendo ba vector director de b->a y bc el vector director b->c

					Waypoint aux_wp;
					double t, aux_speed;

					A = new_points[a];
					B = new_points[b];
					C = new_points[c];

					aux_speed = new_points[b].dSpeed; //takes speed of the waypoint in the middle
					vPoints.pop_back();			// Eliminamos el waypoint del medio. El primer waypoint no se modifica

					for(int j=1; j <= BEZIER_CONTROL_POINTS; j++) {
						t = (double) j * Kt;
						aux_wp = PosOnQuadraticBezier(A, B, C,  t);
						aux_wp.dSpeed = aux_speed;
						vPoints.push_back(aux_wp);
					//	std::cout << "\tWaypointRoute::Optimize: (Bezier) Waypoint,X= " << aux_wp.dX << " Y= " << aux_wp.dY
					//				<< " size= " << (int)points.size() << endl;
					}
				}else{	//Se queda como está

					vPoints.push_back(new_points[c]);
				}
			}

			iCurrentWaypoint = 0;

		//	for(int i = 0; i < new_points.size(); i++){
		//	points.push_back(new_points[i]);
		//	}

		pthread_mutex_unlock(&mutexPath);

		bOptimized  = true;
		new_points.clear();

		return OK;
	}
	//! Prints all the waypoints
	void Print(){
		cout << "Path::Print: Printing all the waypoints..." << endl;
		if(vPoints.size() > 0){
			for(int i = 0; i < vPoints.size(); i++){
				cout << "(" << i << ")\t " << vPoints[i].dX << ", " << vPoints[i].dY << endl;
			}
		}else
			cout << "Path::Print: No waypoints..." << endl;
	}

};


class purepursuit_planner_node: public Component
{

private:	
	ros::NodeHandle node_handle_;
	ros::NodeHandle private_node_handle_;
	double desired_freq_;
	//! constant for Purepursuit
	double Kr;
	
	//! Variable lookahead
    double dLookAhead;
     //! Object with the current path that the robot is following
    Path pathCurrent;
    //! Object with the path that is being filled
    Path pathFilling;
    //! Vector with next paths to follow
    queue <Path> qPath;
    //! current robot's position 
    geometry_msgs::Pose2D pose2d_robot;
    //! current robot's odometry
    nav_msgs::Odometry odometry_robot;
    //! current robot's linear speed
    double dLinearSpeed;
    //! Lookahead bounds
    double d_lookahear_min_, d_lookahear_max_;
    //! Distance from the robot center to the wheel's center
    double d_dist_wheel_to_center_;
    //! Max allowed speed
    double max_speed_;
    //! Flag to enable/disable the motion
    bool bEnabled;
    //! Flag to cancel the following path
    bool bCancel;
    //! Mode for reading the position of the robot ("ODOM", "MAP")
    std::string position_source_;
    //! Mode in numeric format
    unsigned int ui_position_source;
    
	//////// ROS
	//! Publishes the status of the robot
	ros::Publisher status_pub_;
	//! Publish to cmd vel (Ackermann)
	//! It will publish into command velocity (for the robot)
	ros::Publisher vel_pub_;
	//! publish the transformation between map->base_link
	ros::Publisher tranform_map_pub_;
	//! it subscribes to /odom
	ros::Subscriber odom_sub_;
	//! Topic name to read the odometry from
	std::string odom_topic_;
	//! Topic name to publish the vel & pos commands
	std::string cmd_topic_vel_;
	// DIAGNOSTICS
	//! Diagnostic to control the frequency of the published odom
	diagnostic_updater::TopicDiagnostic *updater_diagnostic_odom; 
	//! General status diagnostic updater
	diagnostic_updater::Updater updater_diagnostic;	
	//! Diagnostics min & max odometry freq
	double min_odom_freq, max_odom_freq; 
	//! Saves the time whenever receives a command msg and a transform between map and base link (if configured)
	ros::Time last_command_time, last_map_time;
	// ACTIONLIB
	actionlib::SimpleActionServer<planner_msgs::GoToAction> action_server_goto;
	planner_msgs::GoToFeedback goto_feedback;
	planner_msgs::GoToResult goto_result;
	planner_msgs::GoToGoal goto_goal;
	// TFs
	tf::TransformListener listener;
	tf::StampedTransform transform;
	// SERVICES
	//! service name to enable disable lasers
	std::string name_sc_enable_front_laser_, name_sc_enable_back_laser_;
	//! Service to enable/disable front laser
	ros::ServiceClient sc_enable_front_laser_;
	//! Service to enable/disable back laser
	ros::ServiceClient sc_enable_back_laser_;
	
public:
	
	/*!	\fn summit_controller::purepursuit_planner()
	 * 	\brief Public constructor
	*/
	purepursuit_planner_node(ros::NodeHandle h) : node_handle_(h), private_node_handle_("~"),
	desired_freq_(100.0),Component(desired_freq_),action_server_goto(node_handle_, ros::this_node::getName(), false)
	// boost::bind(&purepursuit_planner_node::executeCB, this, _1), false)
	{
		bRunning = false;
		
		ROSSetup();
		
		dLookAhead = d_lookahear_min_;
		dLinearSpeed = 0;
		pose2d_robot.x = pose2d_robot.y = pose2d_robot.theta = 0.0;
		bEnabled = true;
		bCancel = false;
		
		sComponentName.assign("purepursuit_planner_node");
		iState = INIT_STATE;
	}

	/*!	\fn purepursuit_planner::~purepursuit_planner()
	 * 	\brief Public destructor
	*/
	~purepursuit_planner_node(){	
		
	}

	/*!	\fn oid ROSSetup()
	 * 	\brief Setups ROS' stuff
	*/
	void ROSSetup(){
		private_node_handle_.param<std::string>("odom_topic", odom_topic_, "/odom");
		private_node_handle_.param("cmd_topic_vel", cmd_topic_vel_, std::string("/agvs_controller/command"));
		private_node_handle_.param("d_lookahear_min", d_lookahear_min_, D_LOOKAHEAD_MIN);
		private_node_handle_.param("d_lookahear_max", d_lookahear_max_, D_LOOKAHEAD_MAX);
		private_node_handle_.param("d_dist_wheel_to_center", d_dist_wheel_to_center_, D_WHEEL_ROBOT_CENTER);
		private_node_handle_.param("max_speed", max_speed_, MAX_SPEED);
		private_node_handle_.param("kr", Kr, AGVS_DEFAULT_KR);
		private_node_handle_.param<std::string>("position_source", position_source_, "ODOM");
		private_node_handle_.param("desired_freq", desired_freq_, desired_freq_);
		//private_node_handle_.param<std::string>("name_sc_enable_frot_laser_", name_sc_enable_front_laser_, "/s3000_laser_front/enable_disable");
		//private_node_handle_.param<std::string>("name_sc_enable_back_laser", name_sc_enable_back_laser_, "/s3000_laser_back/enable_disable"	);
		
	
		// From Component class
		threadData.dDesiredHz = desired_freq_;
		
		if(position_source_ == "MAP")
			ui_position_source = MAP_SOURCE;
		else 
			ui_position_source = ODOM_SOURCE;
			
		//
		// Publish through the node handle Twist type messages to the guardian_controller/command topic
		vel_pub_ = private_node_handle_.advertise<ackermann_msgs::AckermannDriveStamped>(cmd_topic_vel_, 1);
		//
		if(ui_position_source == MAP_SOURCE)
			tranform_map_pub_ = private_node_handle_.advertise<geometry_msgs::TransformStamped>("map_location", 100);
		//status_pub_ = private_node_handle_.advertise<purepursuit_planner::ControllerStatus>("status", 1);
		odom_sub_ = private_node_handle_.subscribe<nav_msgs::Odometry>(odom_topic_, 1, &purepursuit_planner_node::OdomCallback, this ); 
		//cmd_vel_sub_ = private_node_handle_.subscribe<purepursuit_planner::AckermannDriveStamped>("command", 1, &purepursuit_planner_node::CmdVelCallback, this ); 
		
		// Diagnostics
		updater_diagnostic.setHardwareID("PurePursuit-Planner");
		// Topics freq control 
		min_odom_freq = 5.0;
		max_odom_freq = 100.0;
		updater_diagnostic_odom = new diagnostic_updater::TopicDiagnostic(odom_topic_, updater_diagnostic,
							diagnostic_updater::FrequencyStatusParam(&min_odom_freq, &max_odom_freq, 0.1, 10),
							diagnostic_updater::TimeStampStatusParam(0.001, 0.1));
		
		
		// Action server 
		action_server_goto.registerGoalCallback(boost::bind(&purepursuit_planner_node::GoalCB, this));
        action_server_goto.registerPreemptCallback(boost::bind(&purepursuit_planner_node::PreemptCB, this));
        
        // Services
        //sc_enable_front_laser_ = private_node_handle_.serviceClient<s3000_laser::enable_disable>(name_sc_enable_front_laser_);
        //sc_enable_back_laser_ = private_node_handle_.serviceClient<s3000_laser::enable_disable>(name_sc_enable_back_laser_);
        
		ROS_INFO("%s::ROSSetup(): odom_topic = %s, command_topic_vel = %s, position source = %s, desired_hz=%.1lf, min_lookahead = %.1lf, max_lookahead = %.1lf, kr = %.2lf", sComponentName.c_str(), odom_topic_.c_str(),
		 cmd_topic_vel_.c_str(), position_source_.c_str(), desired_freq_, d_lookahear_min_, d_lookahear_max_, Kr);
		
		//ROS_INFO("%s::ROSSetup(): laser_topics: front -> %s, back -> %s", sComponentName.c_str(), name_sc_enable_front_laser_.c_str(), name_sc_enable_back_laser_.c_str());
		
		//last_command_time = ros::Time::now();
	}
	
	
	/*!	\fn ReturnValue Setup()
	 * 	\brief 
	*/
	ReturnValue Setup(){
		// Checks if has been initialized
		if(bInitialized){
			ROS_INFO("purepursuit_planner::Setup: Already initialized");	
			return INITIALIZED;
		}
		
		// Starts action server 
		action_server_goto.start();
		
		bInitialized = true;
		
		return OK;
	}
	

	/*! \fn int ReadAndPublish()
	 * Reads data a publish several info into different topics
	*/
	int ReadAndPublish()
	{
		//updater_diagnostic_odom->tick(ros::Time::now());
		updater_diagnostic.update();
		return(0);
	}
	
	/*!	\fn ReturnValue Start()
	 * 	\brief Start Controller
	*/
	ReturnValue Start(){

		if(bRunning){
			ROS_INFO("agvs_controller::Start: the component's thread is already running");
			return THREAD_RUNNING;
		}
		
		
		bRunning = true;
		return OK;
	}

	/*!	\fn ReturnValue Stop()
	 * 	\brief Stop Controller
	*/
	ReturnValue Stop(){
		
		if(!bRunning){
			ROS_INFO("agvs_controller::Stop: Thread not running");
		
			return THREAD_NOT_RUNNING;
		}
		
		bRunning = false;
		
		return OK;
	}

	
	/*! \fn void ControlThread()
	*/
	void ControlThread()
	{
		ROS_INFO("purepursuit_planner::ControlThread(): Init");
		ros::Rate r(desired_freq_);  // 50.0 

			
		// while(node_handle_.ok()) {
		while(ros::ok()) {
			
			switch(iState){
				
				case INIT_STATE:
					InitState();
				break;
				
				case STANDBY_STATE:
					StandbyState();
				break;
				
				case READY_STATE:
					ReadyState();
				break;
				
				case SHUTDOWN_STATE:
					ShutDownState();
				break;
				
				case EMERGENCY_STATE:
					EmergencyState();
				break;
				
				case FAILURE_STATE:
					FailureState();
				break;
			
			}
			
			AllState();
			
			ros::spinOnce();
			r.sleep();
		}
		ShutDownState();

		ROS_INFO("purepursuit_planner::ControlThread(): End");
		
	}
	
	
	/*!	\fn void InitState()
	*/
	void InitState(){
		//ROS_INFO("purepursuit_planner::InitSate:");
		if(bInitialized && bRunning){
			if(CheckOdomReceive() == 0)
				SwitchToState(STANDBY_STATE);
		}else{
			if(!bInitialized)
				Setup();
			if(!bRunning)
				Start();
		}		
		
	}
	
	/*!	\fn void StandbyState()
	*/
	void StandbyState(){
		if(CheckOdomReceive() == -1)
			SwitchToState(EMERGENCY_STATE);
		else{
			if(bEnabled && !bCancel ){
				
				if(pathCurrent.Size() > 0 || MergePath() == OK){
					ROS_INFO("%s::StandbyState: route available", sComponentName.c_str());
					SwitchToState(READY_STATE);
				}
			}
				
		}
	}
	
	/*!	\fn void ReadyState()
	*/
	void ReadyState(){
		if(CheckOdomReceive() == -1){
			SetRobotSpeed(0.0, 0.0);
			SwitchToState(EMERGENCY_STATE);
			return;
		}
		if(!bEnabled){
			ROS_INFO("%s::ReadyState: Motion is disabled", sComponentName.c_str());
			SetRobotSpeed(0.0, 0.0);
			SwitchToState(STANDBY_STATE);
			return;
		}
		if(bCancel){
			ROS_INFO("%s::ReadyState: Cancel requested", sComponentName.c_str());
			SetRobotSpeed(0.0, 0.0);
			SwitchToState(STANDBY_STATE);
			return;
		}
		
		int ret = PurePursuit();
		
		if(ret == -1){
			ROS_ERROR("%s::ReadyState: Error on PurePursuit", sComponentName.c_str());
			bCancel = true;	//Activates the flag to cancel the mision
			SetRobotSpeed(0.0, 0.0);
			goto_result.route_result = -1;
			goto_feedback.percent_complete = 100.0;	// Set the percent to 100% to complete the action
			
			SwitchToState(STANDBY_STATE);
			
		}else if(ret == 1){
			ROS_INFO("%s::ReadyState: Route finished", sComponentName.c_str());
			SetRobotSpeed(0.0, 0.0);
			goto_result.route_result = 0;
			goto_feedback.percent_complete = 100.0;	// Set the percent to 100% to complete the action
			SwitchToState(STANDBY_STATE);
		}
		// We have to update the percent while the mision is ongoing
		
	}
	
	
	/*! \fn void UpdateLookAhead()
	*   \brief Updates (little by little) the variable lookahead depending of the current velocity
	*/
	void UpdateLookAhead(){
		double aux_lookahead = fabs(dLinearSpeed);
		double desired_lookahead = 0.0;
		double inc = 0.01;	// incremento del lookahead
		
		if(aux_lookahead < d_lookahear_min_)
			desired_lookahead = d_lookahear_min_;
		else if(aux_lookahead > d_lookahear_max_)
			desired_lookahead = d_lookahear_max_;
		else{
			desired_lookahead = aux_lookahead;
		}

		if((desired_lookahead - 0.001) > dLookAhead){
			dLookAhead+= inc;
		}else if((desired_lookahead + 0.001) < dLookAhead)
			dLookAhead-= inc;
	}
	
	
	/*! \fn double Dot2( double x1, double y1, double x2, double y2)
	*   \brief Obtains vector cross product w x v
	*   \return w.x * v.x + w.y * w.y
	*/
	double Dot2( double x1, double y1, double x2, double y2) {
		return (x1*x2 + y1*y2); // cross product
	}


	/*! \fn double Dist(double x1, double y1, double x2, double y2)
	*   \brief obtains distance between points p1 and p2
	*/
	double Dist(double x1, double y1, double x2, double y2) {
		double diff_x = (x2 - x1);
		double diff_y = (y2 - y1);
		return sqrt( diff_x*diff_x + diff_y*diff_y );
	}

	/*! \fn double DistP2S( Odometry current_position, Waypoint s0, Waypoint s1, Waypoint *Pb)
	 *  \brief obtains distance between the current position and segment s0->s1, and returns the point
	 *	Return: the shortest distance from p to s (utm points) and the point
	 *	of the segment that gives the shortest distance
	*/
	double DistP2S( geometry_msgs::Pose2D current_position, Waypoint s0, Waypoint s1, Waypoint *Pb){
		double vx,vy, wx, wy;

		double c1, c2, di, b;

		vx = s1.dX - s0.dX;
		vy = s1.dY - s0.dY;

		wx = current_position.x - s0.dX;
		wy = current_position.y - s0.dY;

		c1 = Dot2( wx, wy, vx, vy );

		if ( c1 <= 0.0 ) {
			di = Dist(current_position.x, current_position.y, s0.dX, s0.dY);
			Pb->dX = s0.dX;
			Pb->dY = s0.dY;
			return di;
		}

		c2 = Dot2(vx,vy, vx, vy);

		if ( c2 <= c1 ) {
			//printf("kanban::DistP2S: c2 <= c1\n");
			di = Dist(current_position.x, current_position.y, s1.dX, s1.dY);
			Pb->dX = s1.dY;
			Pb->dY = s1.dY;
			return di;
		}

		b = c1 / c2;
		Pb->dX = s0.dX + b * vx;
		Pb->dY = s0.dY + b * vy;

		di = Dist(current_position.x, current_position.y, Pb->dX, Pb->dY);

		return di;
	}
	

	/*! \fn ReturnValue PointDlh(geometry_msgs::Pose2D current_position, geometry_msgs::Pose2D *wp	)
	 *  \brief Returns a point in a distance dlookahead on the path
	 *  \return OK
	 *  \return ERROR
	*/
	ReturnValue PointDlh(geometry_msgs::Pose2D current_position, geometry_msgs::Pose2D *wp) {
		int i,j=0,k;
		double dmin, d, d1, d2, *d_seg;
		double t;
		geometry_msgs::Pose2D target_position;
		Waypoint s0, s1, Pb, Pb1;

		int size = pathCurrent.NumOfWaypoints();

		d_seg = new double[size]; //array con la distancia entre puntos consecutivos en la ruta

		// 1- Find closest segment
		dmin = 100000000;

		for(i = pathCurrent.GetCurrentWaypointIndex(); i < (size -1); i++) {
			if( (pathCurrent.GetWaypoint(i, &s0) == OK) &&  (pathCurrent.GetWaypoint(i+1, &s1) == OK) ){
				d_seg[i] = Dist(s0.dX, s0.dY, s1.dX, s1.dY);

				d = DistP2S(current_position, s0, s1, &Pb1);		// Pb1 closest point on segment
				
				if (d < dmin) {
					Pb.dX = Pb1.dX;  // not the same as Pb=Pb1 !
					Pb.dY = Pb1.dY;
					dmin = d;
					j = i;      // j : index to closest segment					
				}
				//ROS_INFO("PointDlh. Distance to segment %d(%.2lf, %2.lf)->%d(%.2lf, %2.lf) = %.3lf,point (%.3lf, %.3lf) (DMIN = %.3lf)", i,
				//s0.dX, s0.dY, i+1, s1.dX, s1.dY, d, Pb1.dX, Pb1.dY, dmin);
			}else{
				ROS_ERROR("%s::PointDlh: Error Getting waypoints", sComponentName.c_str());
				return ERROR;
			}
		}
		
		//ROS_INFO("PointDlh:: Current waypoint index %d, next %d",pathCurrent.GetCurrentWaypointIndex(), j);
				
		// Si cambiamos de waypoint
		if(pathCurrent.GetCurrentWaypointIndex() != j){
			// Sets the waypoint where the robot is at the moment
			if(pathCurrent.SetCurrentWaypoint(j) == ERROR){
				ROS_ERROR("%s::PointDlh: Error setting current waypoint to %d", sComponentName.c_str(), j);
				return ERROR;
			}else{ // OK
				//ROS_INFO("PointDlh:: Changing waypoint to %d", j);
				if(j == (size - 2)){	// Penultimo waypoint
					Waypoint w_last, w_before_last;
					pathCurrent.GetCurrentWaypoint(&w_before_last);	// Penultimo waypoint
					pathCurrent.BackWaypoint(&w_last);				// Ultimo waypoint
					// Distancia maxima = distancia entre el punto actual y el penultimo punto + más la distancia entre los dos ultimos puntos + un valor constante
					//dMaxDistance = Dist(w_before_last.dX, w_before_last.dY, odomWhenLastWaypoint.px, odomWhenLastWaypoint.py) + Dist(w_last.dX, w_last.dY, w_before_last.dX, w_before_last.dY) + 0.1;
					//ROS_INFO("%s::PointDlh: Penultimo punto. Robot en (%.3lf, %.3lf, %.3lf). Distancia máxima a recorrer = %.3lf m ", sComponentName.c_str(), odomWhenLastWaypoint.px, odomWhenLastWaypoint.py, odomWhenLastWaypoint.pa, dMaxDistance);
				}

			}
		}

		// 2-Find segment ahead in dlookahead
		if( pathCurrent.GetNextWaypoint(&s1) != OK ){
			ROS_ERROR("%s::PointDlh: Error getting next waypoint %d", sComponentName.c_str(), j);
			return ERROR;
		}

		d1 = Dist(Pb.dX, Pb.dY, s1.dX, s1.dY);

		k = j;              // k : index of D_LOOKAHEAD point segment
		while ( (d1 < dLookAhead) && ( (k+1) < (size - 1) ) ) {
			// searched point on this segment
			k = k + 1;
			d1 = d1 + d_seg[k];
		}

		// 3- Obtain t parameter in the segment
		d2 = ( d1 - dLookAhead );       // t parameter of segment k
		t = (d_seg[k] - d2) / d_seg[k]; // Pendiente avoid div/0. En teoria no puede producirse pq dos waypoints consecutivos serán diferentes
		
		// 4- Obtain point with t parameter
		if( (pathCurrent.GetWaypoint(k, &s0) == OK) && (pathCurrent.GetWaypoint(k + 1, &s1) == OK) ) {

			target_position.x = s0.dX + ( s1.dX - s0.dX )*t; 
			target_position.y = s0.dY + ( s1.dY - s0.dY )*t;
			double angle_segment = atan2(s1.dY - s0.dY, s1.dX - s0.dX);

			radnorm(&angle_segment);

			target_position.theta = angle_segment;
			*wp = target_position;
			
			delete d_seg;

			return OK;
		}else{
			ROS_ERROR("%s::PointDlh: Error getting next waypoint %d", sComponentName.c_str(), j);
			return ERROR;
		}

	}


	/*!	\fn int PurePursuit()
	 * \brief High level control loop in cartesian coordinates
	 * obtains desiredSpeedMps and desiredPhiEffort according to
	 * the robot location and the path defined by the waypoints
	 *  \return 0 if the iteration is OK
	 *  \return -1 if there's a problem
	 *  \return 1 if the route finishes
	 */
	int PurePursuit(){
		double dx, dy, x1, y1;
		double curv, yaw;
		double wref;//, epw, uw;
		//double d = D_WHEEL_ROBOT_CENTER;   // Length in m (equiv to curv radius)
		double Kd = 1.1; // don't increase! 250
		Waypoint last_waypoint, next_waypoint;
		
		double dAuxSpeed = 0.0;
		double dth;
		double aux = 0.0, dDistCovered = 0.0;
		int ret = 0;
		
		geometry_msgs::Pose2D current_position = this->pose2d_robot;		
		geometry_msgs::Pose2D next_position;		
		
		if(pathCurrent.NumOfWaypoints() < 2)	{
			ROS_ERROR("%s::PurePursuit: not enought waypoints", sComponentName.c_str());			
			return -1 ;
		}

		yaw = current_position.theta;
		
		//
		//Updates the lookahead depending of the current velocity
		UpdateLookAhead();
		
		//
		// Get next point in cartesian coordinates
		if(PointDlh(current_position, &next_position) != OK){
			ROS_ERROR("%s::PurePursuit: Error getting next point in the route", sComponentName.c_str());
			return -1;
		}
		//
		// Curvature
		dx = current_position.x - next_position.x;
		dy = current_position.y - next_position.y;
		x1 = cos(yaw)*dx + sin(yaw)*dy; //Original
		y1 = -sin(yaw)*dx + cos(yaw)*dy;

		if ((x1*x1 + y1*y1) == 0)
			curv = 0;
		else
			curv = (2.0 / (x1*x1 + y1*y1)) * -y1;  		//Original

		// Obtenemos alfa_ref en bucle abierto segun curvatura
		wref = atan(d_dist_wheel_to_center_/(1.0/curv));		
		
		
		if(pathCurrent.BackWaypoint(&last_waypoint) == ERROR){
			ROS_ERROR("%s::PurePursuit: Error getting the last point in the path", sComponentName.c_str());
			return -1;
		}

		double dAuxDist = Dist(current_position.x, current_position.y, last_waypoint.dX, last_waypoint.dY);	//dist(waypoints.back().pos, current_position);

		if(pathCurrent.GetNextWaypoint(&next_waypoint) == ERROR){
			ROS_ERROR("%s::PurePursuit: Error getting next waypoint in the path", sComponentName.c_str());
			return -1;
		}

		dAuxSpeed = next_waypoint.dSpeed;

		if (dAuxSpeed >= 0)
		   dth = next_position.theta - current_position.theta;
		else
		   dth = -(next_position.theta + Pi - current_position.theta);

		// normalize
		radnorm(&dth);
		double aux_wref = wref;
		wref += Kr * dth;
		
		//ROS_INFO("Purepursuit: current pos (%.2lf, %.2lf), next pos (%.2lf, %.2lf), lookahead %.2lf, yaw = %.3lf, curv = %.3lf, dth = %.3lf, wref = %.3lf(%.3lf), speed=%.3lf", current_position.x, current_position.y, next_position.x, next_position.y, dLookAhead, yaw, curv, dth, wref, aux_wref, dAuxSpeed);
		//ROS_INFO("Purepursuit: yaw = %.3lf, curv = %.3lf, dth = %.3lf, wref = %.3lf", yaw, curv, dth, wref);
		
		
		////////////////// Sets the speed depending of distance or speed restrictions /////////
		///////////////////////////////////////////////////////////////////////////////////////
		// Controls the max allowed using first restriction
		if(fabs(dAuxSpeed) > max_speed_){ 
			if(dAuxSpeed > 0)
				dAuxSpeed = max_speed_;
			else
				dAuxSpeed = -max_speed_;
		}
			
		
		if(dAuxDist <= AGVS_SECOND_DECELERATION_DISTANCE)	{
			if( (dAuxSpeed < 0.0) && (dAuxSpeed < -AGVS_SECOND_DECELERATION_MAXSPEED) )
				dAuxSpeed = -AGVS_SECOND_DECELERATION_MAXSPEED;
			else if( (dAuxSpeed > 0.0) && (dAuxSpeed > AGVS_SECOND_DECELERATION_MAXSPEED) )
				dAuxSpeed = AGVS_SECOND_DECELERATION_MAXSPEED;

		}else if(dAuxDist <= AGVS_FIRST_DECELERATION_DISTANCE) {
			if( (dAuxSpeed < 0.0) && (dAuxSpeed < AGVS_FIRST_DECELERATION_MAXSPEED))
				dAuxSpeed = -AGVS_FIRST_DECELERATION_MAXSPEED;
			else if( (dAuxSpeed > 0.0) && (dAuxSpeed > AGVS_FIRST_DECELERATION_MAXSPEED) )
				dAuxSpeed = AGVS_FIRST_DECELERATION_MAXSPEED;
		}
		
		SetRobotSpeed(dAuxSpeed, wref); 
		
		//
		// When the robot is on the last waypoint, checks the distance to the end
		if( pathCurrent.GetCurrentWaypointIndex() >= (pathCurrent.NumOfWaypoints() - 2) ){
			ret = -10;
			double ddist2 = Dist( current_position.x, current_position.y, last_waypoint.dX, last_waypoint.dY);
			// Distancia recorrida
			//dDistCovered = Dist( current_position.px, current_position.py, odomWhenLastWaypoint.px, odomWhenLastWaypoint.py);
			if (ddist2 < WAYPOINT_POP_DISTANCE_M) {
				SetRobotSpeed(0.0, 0.0);
				
				ROS_INFO("%s::PurePursuit: target position reached (%lf, %lf, %lf). Ending current path", sComponentName.c_str(), current_position.x, current_position.x, current_position.theta*180.0/Pi);
				
				pathCurrent.Clear();
				return 1;
			}
		}

		return 0;
	}
	
	/*!	\fn void CancelPath()
	 * Removes all the waypoints introduced in the system
	*/
	void CancelPath(){
		
		pathCurrent.Clear();	// Clears current path
		pathFilling.Clear();	// Clears the auxiliary path
		while(!qPath.empty())	// Clears the queue of paths
			qPath.pop();
		
		bCancel = false;
		// Cancels current action
		ROS_INFO("%s::CancelPath: action server preempted", sComponentName.c_str());
		action_server_goto.setPreempted();
	}
	
	/*!	\fn void SetRobotSpeed()
	*/
	void SetRobotSpeed(double speed, double angle){
		ackermann_msgs::AckermannDriveStamped ref_msg;
		
		ref_msg.header.stamp = ros::Time::now();
		ref_msg.drive.jerk = 0.0; 
		ref_msg.drive.acceleration = 0.0; 
		ref_msg.drive.steering_angle_velocity = 0.0;
		ref_msg.drive.steering_angle = angle;
		ref_msg.drive.speed = speed;
		
		vel_pub_.publish(ref_msg);
	}
	
	
	/*!	\fn void ShutDownState()
	*/
	void ShutDownState(){
		if(bRunning)
			Stop();
		else if(bInitialized)
			ShutDown();
			
	}
	
	/*!	\fn void EmergencyState()
	*/
	void EmergencyState(){
		if(CheckOdomReceive() == 0){
			SwitchToState(STANDBY_STATE);
			return;
		}
		
	}
	
	/*!	\fn void FailureState()
	*/
	void FailureState(){
	
	}
	
	/*!	\fn void AllState()
	*/
	void AllState(){
		
		// Only if we use the map as source for positioning
		if(ui_position_source == MAP_SOURCE){		
			try{
				listener.lookupTransform("/map", "/base_link", ros::Time(0), transform);
				geometry_msgs::TransformStamped msg;
				tf::transformStampedTFToMsg(transform, msg);
				pose2d_robot.x = msg.transform.translation.x;
				pose2d_robot.y = msg.transform.translation.y;
				pose2d_robot.theta = tf::getYaw(msg.transform.rotation);  
				// Safety check
				last_map_time = ros::Time::now();
				msg.header.stamp = last_map_time;
				tranform_map_pub_.publish(msg);
			}catch (tf::TransformException ex){
				//ROS_ERROR("%s::AllState: %s", sComponentName.c_str(), ex.what());
			}
		}
		
		AnalyseCB();	// Checks action server state
		
		ReadAndPublish();	// Reads and publish into configured topics
		
		if(bCancel)		// Performs the cancel in case of required
			CancelPath();
	}
	
	/*! \fn void OdomCallback(const nav_msgs::Odometry::ConstPtr& odom_value)
		* Receives odom values
	*/
	void OdomCallback(const nav_msgs::Odometry::ConstPtr& odom_value)
	{
		// Safety check
		last_command_time = ros::Time::now();
		
		// If we want to use the odom source, subscribes to odom topic
		if(ui_position_source == ODOM_SOURCE){		
			// converts the odom to pose 2d
			pose2d_robot.x = odom_value->pose.pose.position.x;
			pose2d_robot.y = odom_value->pose.pose.position.y;
			pose2d_robot.theta = tf::getYaw(odom_value->pose.pose.orientation);
		}
		
		// Copies the current odom
		odometry_robot = *odom_value;
		// Gets the linear speed
		dLinearSpeed = odometry_robot.twist.twist.linear.x;
	}
	
	/*! \fn int CheckOdomReceive()
		* Checks whether or not it's receiving odom values and/or map transformations
		* \return 0 if OK
		* \return -1 if ERROR
	*/
	int CheckOdomReceive()
	{		
		// Safety check
		if((ros::Time::now() - last_command_time).toSec() > ODOM_TIMEOUT_ERROR)
			return -1;
		else{
			if( ui_position_source == MAP_SOURCE and ((ros::Time::now() - last_map_time).toSec() > MAP_TIMEOUT_ERROR))
				return -1;
			else return 0;
		}
		
	}
	
	
	void executeCB(const planner_msgs::GoToGoalConstPtr &goal)
	{
		
	}
	
	/*! \fn int CalculateDirectionSpeed(Waypoint target_position)
	*	\brief Calcula el sentido de movimiento de una ruta, dependiendo de la posición inicial y el ángulo del robot
	*	\return 1 si el sentido es positivo
	*	\return -1 si el sentido es negativo
	*/
	int CalculateDirectionSpeed(Waypoint target_position){
		int ret = 1;
		double alpha = pose2d_robot.theta;
		double x =	pose2d_robot.x, y = pose2d_robot.y;
		double ux, uy, vx, vy;
		double beta = 0.0;
		static int last_direction = 0;
		static double pi_medios = M_PI / 2.0, max_diff = 5.0 * M_PI / 180.0;
		int iCase = 0;

		//
		// si la posicion objetivo es la misma del robot, devolvemos 0
		if( (target_position.dX == x) && (target_position.dY == y) ){
			return 0;
		}
		// Cálculo del vector director del robot respecto al sistema de coordenadas del robot
		ux = cos(alpha);
		uy = sin(alpha);
		// Cálculo del vector entre el punto objetivo y el robot
		vx = target_position.dX - x;
		vy = target_position.dY - y;

		// Cálculo del ángulo entre el vector director y el vector al punto objetivo
		beta = acos( (ux * vx + uy * vy) / ( sqrt(ux*ux + uy*uy) * sqrt(vx*vx + vy*vy) ) );

		// Devolvemos valor dependiendo del ángulo entre la orientación del robot y la posición objetivo (radianes)
		// Tendremos en cuenta el valor del sentido de avance de la última ruta.
		if(fabs(beta) <= pi_medios){
			// Calculo inicial de direccion
			if(last_direction == 0)
				ret = 1;
			else {
				ret = 1;

				if( fabs(beta - pi_medios) <= max_diff){
					if(last_direction != ret){
						iCase = 1;
						ret = -1;

					}else {
						iCase = 2;
					}
				}

			}
		}else{
			// Calculo inicial de direccion
			if(last_direction == 0)
				ret = -1;
			else {
				ret = -1;
				if(fabs(beta - pi_medios) <= max_diff){
					if(last_direction != ret){
						ret = 1;
						iCase = 3;
					}else{
						iCase = 4;
					}
				}

			}
		}

		ROS_INFO("%s:CalculateDirectionSpeed:  case %d. Beta = %.2lf. Diff = %.2lf. Last direction = %d, new direction = %d", sComponentName.c_str(),
				 iCase, beta*180.0/M_PI, (beta - pi_medios)*180.0/M_PI, last_direction, ret);

		last_direction = ret;
		return ret;
	}
	
	/*! \fn int CalculateDirectionSpeed(geometry_msgs::Pose2D target_position)
	*	\brief Calcula el sentido de movimiento de una ruta, dependiendo de la posición inicial y el ángulo del robot
	*	\return 1 si el sentido es positivo
	*	\return -1 si el sentido es negativo
	*/
	int CalculateDirectionSpeed(geometry_msgs::Pose2D target_position){
		int ret = 1;
		double alpha = pose2d_robot.theta;
		double x =	pose2d_robot.x, y = pose2d_robot.y;
		double ux, uy, vx, vy;
		double beta = 0.0;
		static int last_direction = 0;
		static double pi_medios = M_PI / 2.0, max_diff = 5.0 * M_PI / 180.0;
		int iCase = 0;

		//
		// si la posicion objetivo es la misma del robot, devolvemos 0
		if( (target_position.x == x) && (target_position.y == y) ){
			return 0;
		}
		// Cálculo del vector director del robot respecto al sistema de coordenadas del robot
		ux = cos(alpha);
		uy = sin(alpha);
		// Cálculo del vector entre el punto objetivo y el robot
		vx = target_position.x - x;
		vy = target_position.y - y;

		// Cálculo del ángulo entre el vector director y el vector al punto objetivo
		beta = acos( (ux * vx + uy * vy) / ( sqrt(ux*ux + uy*uy) * sqrt(vx*vx + vy*vy) ) );

		// Devolvemos valor dependiendo del ángulo entre la orientación del robot y la posición objetivo (radianes)
		// Tendremos en cuenta el valor del sentido de avance de la última ruta.
		if(fabs(beta) <= pi_medios){
			// Calculo inicial de direccion
			if(last_direction == 0)
				ret = 1;
			else {
				ret = 1;

				if( fabs(beta - pi_medios) <= max_diff){
					if(last_direction != ret){
						iCase = 1;
						ret = -1;

					}else {
						iCase = 2;
					}
				}

			}
		}else{
			// Calculo inicial de direccion
			if(last_direction == 0)
				ret = -1;
			else {
				ret = -1;
				if(fabs(beta - pi_medios) <= max_diff){
					if(last_direction != ret){
						ret = 1;
						iCase = 3;
					}else{
						iCase = 4;
					}
				}

			}
		}

		ROS_INFO("%s:CalculateDirectionSpeed:  case %d. Beta = %.2lf. Diff = %.2lf. Last direction = %d, new direction = %d", sComponentName.c_str(),
				 iCase, beta*180.0/M_PI, (beta - pi_medios)*180.0/M_PI, last_direction, ret);

		last_direction = ret;
		return ret;
	}
	
	/*!	\fn ReturnValue Agvs::MergePath()
	 * 	\brief Merges the current path with the next path
	*/
	ReturnValue MergePath(){
		Waypoint new_waypoint, wFirst, wLast;
		Path aux;
		int direction = 0;
		
		if(action_server_goto.isNewGoalAvailable()){
			goto_goal.target = action_server_goto.acceptNewGoal()->target; // Reads points from action server
			if(goto_goal.target.size() > 0){
				if(goto_goal.target.size() > 1){	// Tries to use the second point of the route
					direction = CalculateDirectionSpeed(goto_goal.target[1].pose);
				}else{
					direction = CalculateDirectionSpeed(goto_goal.target[0].pose);
				}
				
				if(direction == 1){	// Uses only front laser
					SetLaserFront();
				}else{	// Uses only back laser
					SetLaserBack();
				}
				
				for(int i = 0; i < goto_goal.target.size(); i++){
					ROS_INFO("%s::MergePath: Point %d (%.3lf, %.3lf, %.3lf) speed = %.3lf", sComponentName.c_str(), i,  goto_goal.target[i].pose.x, 
					goto_goal.target[i].pose.y, goto_goal.target[i].pose.theta, goto_goal.target[i].speed  );
					
					new_waypoint.dX = goto_goal.target[i].pose.x;
					new_waypoint.dY = goto_goal.target[i].pose.y;
					new_waypoint.dA = goto_goal.target[i].pose.theta;
					// Depending on the calculated motion direction, applies positive or negative speed
					if(direction == 1){
						new_waypoint.dSpeed = fabs(goto_goal.target[i].speed);
					}else{
						new_waypoint.dSpeed = -fabs(goto_goal.target[i].speed);
					}
					
					pathFilling.AddWaypoint(new_waypoint);							
				}
				if(pathFilling.Optimize(AGVS_TURN_RADIUS) != OK)
					ROS_ERROR("%s::GoalCB: Error optimizing the path", sComponentName.c_str());

				//pathFilling.Print();
				// Adds the new path to the queue
				qPath.push(pathFilling);
				// Clears temporary path object
				pathFilling.Clear();
				
				goto_feedback.percent_complete = 0.0;	// Inits the feedback percentage
				
				// Only if exists any path into the queue
				if(qPath.size() > 0){					
					aux = qPath.front();
					aux.GetWaypoint(0, &wFirst);
					aux.BackWaypoint(&wLast);
					ROS_INFO("%s::MergePath: Adding new %d points from (%.2lf, %.2lf) to (%.2lf, %.2lf) and %d magnets", sComponentName.c_str(), aux.NumOfWaypoints() ,wFirst.dX, wFirst.dY, wLast.dX, wLast.dY, aux.NumOfMagnets());
					ROS_INFO("%s::MergePath: Current number of points = %d and magnets = %d", sComponentName.c_str(), pathCurrent.NumOfWaypoints() , pathCurrent.NumOfMagnets());
					
					// Adds the first path in the queue to the current path
					pathCurrent+=qPath.front();
					
					ROS_INFO("%s::MergePath: New number of points = %d and magnets = %d", sComponentName.c_str(), pathCurrent.NumOfWaypoints() , pathCurrent.NumOfMagnets());
					// Pops the extracted path
					qPath.pop();
					
					goto_goal.target.clear();	// removes current goals
					
					return OK;
				}
			}
		
		}
		
		return ERROR;
	}
	
	/*! \fn void GoalCB()
		* Called when receiving a new target. (ActionServer)
	*/
	void GoalCB()
	{	
		
		
				
	}
	
	/*! \fn void PreemptCB()
		* Called to cancel or replace current mision. (ActionServer)
	*/
	void PreemptCB()
	{	
		bCancel = true;	
	}
	
	/*! \fn void AnalyseCB()
		* Checks the status. (ActionServer)
	*/
	void AnalyseCB(){
		if (!action_server_goto.isActive()){
			//ROS_INFO("%s::AnalyseCB: Not active", sComponentName.c_str());
			return;
		}
		//goto_feedback.percent_complete+=1.0;
		
		action_server_goto.publishFeedback(goto_feedback);
		
		if(goto_feedback.percent_complete == 100.0){
			//action_server_goto.setSucceeded(goto_result);
			action_server_goto.setAborted(goto_result);
			ROS_INFO("%s::AnalyseCB: Action finished", sComponentName.c_str());
		}
	}
	
	/*! \fn void SetLaserFront()
		* Disables laser back, enables laser front
	*/
	bool SetLaserFront(){
		/*s3000_laser::enable_disable srv;
		
		srv.request.value = false;
		sc_enable_back_laser_.call(srv);
		ROS_INFO("%s::SetLaserFront: Setting laser back to false, ret = %d", sComponentName.c_str(), srv.response.ret);
		
		srv.request.value = true;
		sc_enable_front_laser_.call(srv);
		ROS_INFO("%s::SetLaserFront: Setting laser front to true, ret = %d", sComponentName.c_str(), srv.response.ret);*/
	}
	
	/*! \fn void SetLaserBack()
		* Disables laser front, enables laser back
	*/
	bool SetLaserBack(){
		/*s3000_laser::enable_disable srv;
		
		srv.request.value = false;
		sc_enable_front_laser_.call(srv);
		ROS_INFO("%s::SetLaserBack: Setting laser front to false, ret = %d", sComponentName.c_str(), srv.response.ret);
		
		srv.request.value = true;
		sc_enable_back_laser_.call(srv);
		ROS_INFO("%s::SetLaserBack: Setting laser back to true, ret = %d", sComponentName.c_str(), srv.response.ret);*/
	}

}; // class purepursuit_planner_node

// MAIN
int main(int argc, char** argv)
{
    ros::init(argc, argv, "purepursuit_planner_node");
	
	ros::NodeHandle n;		
  	purepursuit_planner_node planner(n);
	
	planner.ControlThread();

	return (0);
}
// EOF
