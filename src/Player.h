/************************************************************************************
 * WrightEagle (Soccer Simulation League 2D)                                        *
 * BASE SOURCE CODE RELEASE 2013                                                    *
 * Copyright (c) 1998-2013 WrightEagle 2D Soccer Simulation Team,                   *
 *                         Multi-Agent Systems Lab.,                                *
 *                         School of Computer Science and Technology,               *
 *                         University of Science and Technology of China            *
 * All rights reserved.                                                             *
 *                                                                                  *
 * Redistribution and use in source and binary forms, with or without               *
 * modification, are permitted provided that the following conditions are met:      *
 *     * Redistributions of source code must retain the above copyright             *
 *       notice, this list of conditions and the following disclaimer.              *
 *     * Redistributions in binary form must reproduce the above copyright          *
 *       notice, this list of conditions and the following disclaimer in the        *
 *       documentation and/or other materials provided with the distribution.       *
 *     * Neither the name of the WrightEagle 2D Soccer Simulation Team nor the      *
 *       names of its contributors may be used to endorse or promote products       *
 *       derived from this software without specific prior written permission.      *
 *                                                                                  *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND  *
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED    *
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE           *
 * DISCLAIMED. IN NO EVENT SHALL WrightEagle 2D Soccer Simulation Team BE LIABLE    *
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL       *
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR       *
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER       *
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,    *
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF *
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.                *
 ************************************************************************************/

#ifndef __Player_H__
#define __Player_H__

#include "Client.h"
#include "DecisionTree.h"
#include "DynamicDebug.h"
#include "Formation.h"
#include "CommandSender.h"
#include "Parser.h"
#include "Thread.h"
#include "UDPSocket.h"
#include "WorldModel.h"
#include "Agent.h"
#include "VisualSystem.h"
#include "Logger.h"
#include "CommunicateSystem.h"
#include "TimeTest.h"
#include "Dasher.h"
#include "Kicker.h"
#include "Tackler.h"
#include <stdio.h>
#include <stdlib.h>
#include <algorithm>    // std::sort
#include <vector>       // std::vector
#include <functional>   // std::bind
#include <map>
#include <vector>
#include <typeinfo> 

class DecisionTree;
class  BeliefState;

class Player: public Client
{
	//DecisionTree *mpDecisionTree; //hereo

public:
    /**
     * 构造函数和析构函数
     */
    bool isPositioned;
    bool IsOccupying;
    bool mpIntransit;
    bool ResetCallOccupy;
    std::map<Vector, int> holemap;
    std::map<int, Vector> playermap;
    std::vector<Vector> holevector;
    std::vector<int> playervector;
    Vector mpTarget = Vector(0,0);

    Player();
    virtual ~Player();

    void Run();
    void SendOptionToServer();

    bool PassPlayersAvailable(){
    	Vector myPosition = mpAgent->GetSelf().GetPos();
    	Vector currentHole = RoundToNearestHole(myPosition);
    	Vector frontup = Vector(currentHole.X()+10, currentHole.Y()-10);
    	Vector backup = Vector(currentHole.X()-10, currentHole.Y()-10);
    	Vector frontdown = Vector(currentHole.X()+10, currentHole.Y()+10);
    	Vector backdown = Vector(currentHole.X()-10, currentHole.Y()+10);

        Vector fronthor = Vector(currentHole.X()+20, currentHole.Y());
        Vector backhor = Vector(currentHole.X()-20, currentHole.Y());
        Vector upvert = Vector(currentHole.X(), currentHole.Y()-20);
        Vector downvert = Vector(currentHole.X(), currentHole.Y()+20);
    	
    	double buffer = 1.0;
    	
    	//TODO: Can be replaced by the IsOccupied function
        //TODO: Return true only if pass is advantageous
    	
    	for(Unum i=4; i<=11; i++){
    		Vector player_pos = mpAgent->GetWorldState().GetTeammate(i).GetPos();
    		if( AreSamePoints(player_pos, frontup, buffer)||
    		    AreSamePoints(player_pos, frontdown, buffer)||
                AreSamePoints(player_pos, backup, buffer)||
                AreSamePoints(player_pos, backdown, buffer)||
                AreSamePoints(player_pos, fronthor, buffer)
                //||
                //AreSamePoints(player_pos, backhor, buffer)||
                //AreSamePoints(player_pos, upvert, buffer)||
                //AreSamePoints(player_pos, downvert, buffer)
    			){
    			std::cout<<"pass available"<<std::endl;
    			return true;
    		}
    	}
    	return false;	
    }


    double DistanceBetweenPoints(double x1,double y1,double x2,double y2){

        double distance = sqrt(pow((x1-x2),2)+pow((y1-y2),2));

        return distance;
    }

    double AngleBetweenPoints(double x1,double y1,double x2,double y2,double x3,double y3){

        double angle = acos((pow(DistanceBetweenPoints(x1,y1,x2,y2),2)+pow(DistanceBetweenPoints(x1,y1,x3,y3),2)-pow(DistanceBetweenPoints(x2,y2,x3,y3),2))/(2*DistanceBetweenPoints(x1,y1,x2,y2)*DistanceBetweenPoints(x1,y1,x3,y3)));
        
        return angle;
    }

    bool compare_first(const std::pair<double,double>& i, const std::pair<double,double>& j)
    {
    return i.first > j.first;
    }


    static bool compare_second(const std::pair<double,double>& i, const std::pair<double,double>& j)
    {
    return i.second > j.second;
    }

    double slope(double x1,double y1,double x2,double y2){

        double slope_of_line = (y2-y1)/(x2-x1);

        return slope_of_line;
    }

    double constant(double x1,double y1,double slope_of_line){

        return (y1-slope_of_line*x1);
    }

    double bisector_line_const(double c1,double c2,double a1,double a2){


        return ((c2-c1)*sqrt((pow(a2,2)+1)*(pow(a1,2)+1)));
    }

    double bisector_line_x_const(double a1,double a2){

        return ((a2*sqrt(pow(a1,2)+1))-(a1*sqrt(pow(a2,2)+1)));
    }

    double bisector_line_y_const(double a1,double a2){


        return (sqrt(pow(a2,2)+1)-sqrt(pow(a1,2)+1));
    }

    double intersecting_point_x(double c1, double c2, double a, double b){

        double x = ((c2*b - c1*a)/((a*a)+(b*b)));
        
        return x;
    }



    double intersecting_point_y(double c1, double c2, double a, double b){

        double y = ((c2*a + c1*b)/((a*a)+(b*b)));
        
        return y;
    }


    void PlaceThePlayers(){
        Vector myPosition = mpAgent->GetSelf().GetPos();
        double mpx = myPosition.X();
        double mpy = myPosition.Y();

        //Vector currentHole = RoundToNearestHole(myPosition);
         std::vector <std::pair<double,double> > opp_position;    
        

        for(Unum i=1;i<=11;i++){

            if((myPosition - mpAgent->GetWorldState().GetOpponent(i).GetPos()).Mod() <=20.0){

            double op_x = mpAgent->GetWorldState().GetOpponent(i).GetPos().X();
            double op_y = mpAgent->GetWorldState().GetOpponent(i).GetPos().Y();

            opp_position.push_back(std::make_pair(op_x,op_y));
    
            }
            
        }

       std::sort(opp_position.begin(),opp_position.end(),compare_second);

        double max_dist =0.0;
        Vector best_hole = Vector(0.0,0.0);
 
        for(int i=0;i<opp_position.size()-1;i++){
            double min_dist = 10000.0;
                    Vector hole_point = Vector(0.0,0.0);

            for(int j=1;j<opp_position.size();j++){

              if((myPosition.X() < opp_position[i].first) && (myPosition.X() < opp_position[j].first)){
                        
                double slope_of_opp1= slope(mpx,mpy,opp_position[i].first,opp_position[i].second);
                double slope_of_opp2= slope(mpx,mpy,opp_position[j].first,opp_position[j].second);

                    double coeff_of_bisector_x = bisector_line_x_const(slope_of_opp1,slope_of_opp2);

                    double coeff_of_bisector_y = bisector_line_y_const(slope_of_opp1,slope_of_opp2);

                    double const_of_opp1 = constant(mpx,mpy,slope_of_opp1);
                    double const_of_opp2 = constant(mpx,mpy,slope_of_opp2);

                    double constant_of_bisector= bisector_line_const(const_of_opp1,const_of_opp2,slope_of_opp1,slope_of_opp2);

                        // find the perpendicular point on the bisector line
                        // calculate the threat

                     

                    for(Unum l=1;l<=11;l++){

                        if((myPosition.X() < mpAgent->GetWorldState().GetOpponent(l).GetPos().X()) && AngleBetweenPoints(mpx,mpy,mpAgent->GetWorldState().GetOpponent(l).GetPos().X(),mpAgent->GetWorldState().GetOpponent(l).GetPos().Y(),mpAgent->GetWorldState().GetOpponent(i).GetPos().X(),mpAgent->GetWorldState().GetOpponent(i).GetPos().Y())<=90.0) {


                            double const_of_perpendicular = ((mpAgent->GetWorldState().GetOpponent(l).GetPos().X()*coeff_of_bisector_y)+(mpAgent->GetWorldState().GetOpponent(l).GetPos().Y()*coeff_of_bisector_x))/coeff_of_bisector_x; 
                            
                            double corresponding_x = intersecting_point_x(constant_of_bisector,const_of_perpendicular,coeff_of_bisector_x,coeff_of_bisector_y);
                            
                            double corresponding_y = intersecting_point_y(constant_of_bisector,const_of_perpendicular,coeff_of_bisector_x,coeff_of_bisector_y);
                            

                              double dist_mp_intersection = DistanceBetweenPoints(corresponding_x,corresponding_y,mpx,mpy);

                              double dist_op_intersection = DistanceBetweenPoints(corresponding_x,corresponding_y,mpAgent->GetWorldState().GetOpponent(l).GetPos().X(),mpAgent->GetWorldState().GetOpponent(l).GetPos().Y());

                              if(dist_mp_intersection > dist_op_intersection ){

                                    if(dist_mp_intersection < min_dist){

                                        min_dist = dist_mp_intersection;
                                        hole_point = Vector(corresponding_x,corresponding_y);
                                    }

                              }
                                      
                        }

                    }


                }

                    
                if( max_dist < min_dist){
                    max_dist = min_dist;
                    best_hole = Vector(hole_point.X(),hole_point.Y());
            } 

                i=j;
            }
            
            //pick the best place
            //update occupyhole
        }


        max_dist =0.0;
        best_hole = Vector(0.0,0.0);



        for(int i=0;i<opp_position.size()-1;i++){
             double min_dist = 10000.0;
                    Vector hole_point = Vector(0.0,0.0);

            for(int j=1;j<opp_position.size();j++){

              if((myPosition.X() > opp_position[i].first) && (myPosition.X() > opp_position[j].first)){
                        
                double slope_of_opp1= slope(mpx,mpy,opp_position[i].first,opp_position[i].second);
                double slope_of_opp2= slope(mpx,mpy,opp_position[j].first,opp_position[j].second);

                    double coeff_of_bisector_x = bisector_line_x_const(slope_of_opp1,slope_of_opp2);

                    double coeff_of_bisector_y = bisector_line_y_const(slope_of_opp1,slope_of_opp2);

                    double const_of_opp1 = constant(mpx,mpy,slope_of_opp1);
                    double const_of_opp2 = constant(mpx,mpy,slope_of_opp2);

                    double constant_of_bisector= bisector_line_const(const_of_opp1,const_of_opp2,slope_of_opp1,slope_of_opp2);

                        // find the perpendicular point on the bisector line
                        // calculate the threat

                    

                    for(Unum l=1;l<=11;l++){

                        if((myPosition.X() > mpAgent->GetWorldState().GetOpponent(l).GetPos().X()) && AngleBetweenPoints(mpx,mpy,mpAgent->GetWorldState().GetOpponent(l).GetPos().X(),mpAgent->GetWorldState().GetOpponent(l).GetPos().Y(),mpAgent->GetWorldState().GetOpponent(i).GetPos().X(),mpAgent->GetWorldState().GetOpponent(i).GetPos().Y())<=90.0) {


                            double const_of_perpendicular = ((mpAgent->GetWorldState().GetOpponent(l).GetPos().X()*coeff_of_bisector_y)+(mpAgent->GetWorldState().GetOpponent(l).GetPos().Y()*coeff_of_bisector_x))/coeff_of_bisector_x; 
                            
                            double corresponding_x = intersecting_point_x(constant_of_bisector,const_of_perpendicular,coeff_of_bisector_x,coeff_of_bisector_y);
                            
                            double corresponding_y = intersecting_point_y(constant_of_bisector,const_of_perpendicular,coeff_of_bisector_x,coeff_of_bisector_y);
                            

                              double dist_mp_intersection = DistanceBetweenPoints(corresponding_x,corresponding_y,mpx,mpy);

                              double dist_op_intersection = DistanceBetweenPoints(corresponding_x,corresponding_y,mpAgent->GetWorldState().GetOpponent(l).GetPos().X(),mpAgent->GetWorldState().GetOpponent(l).GetPos().Y());

                              if(dist_mp_intersection > dist_op_intersection ){

                                    if(dist_mp_intersection < min_dist){

                                        min_dist = dist_mp_intersection;
                                        hole_point = Vector(corresponding_x,corresponding_y);
                                    }

                              }
                                      
                        }

                    }


                }

                    
                if( max_dist < min_dist){
                    max_dist = min_dist;
                    best_hole = Vector(hole_point.X(),hole_point.Y());
            }

                i=j;
            }
            
            //pick the best place
            //update occupyhole
        }


         max_dist =0.0;
        best_hole = Vector(0.0,0.0);

        std::sort(opp_position.begin(),opp_position.end());


for(int i=0;i<opp_position.size()-1;i++){

double min_dist = 10000.0;
Vector hole_point = Vector(0.0,0.0);

            for(int j=1;j<opp_position.size();j++){

              if((myPosition.Y() > opp_position[i].first) && (myPosition.Y() > opp_position[j].first)){
                        
                double slope_of_opp1= slope(mpx,mpy,opp_position[i].first,opp_position[i].second);
                double slope_of_opp2= slope(mpx,mpy,opp_position[j].first,opp_position[j].second);

                    double coeff_of_bisector_x = bisector_line_x_const(slope_of_opp1,slope_of_opp2);

                    double coeff_of_bisector_y = bisector_line_y_const(slope_of_opp1,slope_of_opp2);

                    double const_of_opp1 = constant(mpx,mpy,slope_of_opp1);
                    double const_of_opp2 = constant(mpx,mpy,slope_of_opp2);

                    double constant_of_bisector= bisector_line_const(const_of_opp1,const_of_opp2,slope_of_opp1,slope_of_opp2);

                        // find the perpendicular point on the bisector line
                        // calculate the threat

                     

                    for(Unum l=1;l<=11;l++){

                        if((myPosition.Y() > mpAgent->GetWorldState().GetOpponent(l).GetPos().Y()) && AngleBetweenPoints(mpx,mpy,mpAgent->GetWorldState().GetOpponent(l).GetPos().X(),mpAgent->GetWorldState().GetOpponent(l).GetPos().Y(),mpAgent->GetWorldState().GetOpponent(i).GetPos().X(),mpAgent->GetWorldState().GetOpponent(i).GetPos().Y())<=90.0) {


                            double const_of_perpendicular = ((mpAgent->GetWorldState().GetOpponent(l).GetPos().X()*coeff_of_bisector_y)+(mpAgent->GetWorldState().GetOpponent(l).GetPos().Y()*coeff_of_bisector_x))/coeff_of_bisector_x; 
                            
                            double corresponding_x = intersecting_point_x(constant_of_bisector,const_of_perpendicular,coeff_of_bisector_x,coeff_of_bisector_y);
                            
                            double corresponding_y = intersecting_point_y(constant_of_bisector,const_of_perpendicular,coeff_of_bisector_x,coeff_of_bisector_y);
                            

                              double dist_mp_intersection = DistanceBetweenPoints(corresponding_x,corresponding_y,mpx,mpy);

                              double dist_op_intersection = DistanceBetweenPoints(corresponding_x,corresponding_y,mpAgent->GetWorldState().GetOpponent(l).GetPos().X(),mpAgent->GetWorldState().GetOpponent(l).GetPos().Y());

                              if(dist_mp_intersection > dist_op_intersection ){

                                    if(dist_mp_intersection < min_dist){

                                        min_dist = dist_mp_intersection;
                                        hole_point = Vector(corresponding_x,corresponding_y);
                                    }

                              }
                                      
                        }

                    }


                }

                    
                if( max_dist < min_dist){
                    max_dist = min_dist;
                    best_hole = Vector(hole_point.X(),hole_point.Y());
            }

                i=j;
            }
            
            //pick the best place
            //update occupyhole
        }


         max_dist =0.0;
         best_hole = Vector(0.0,0.0);

        for(int i=0;i<opp_position.size()-1;i++){
            double min_dist = 10000.0;
            Vector hole_point = Vector(0.0,0.0); 


            for(int j=1;j<opp_position.size();j++){

              if((myPosition.Y() < opp_position[i].first) && (myPosition.Y() < opp_position[j].first)){
                        
                double slope_of_opp1= slope(mpx,mpy,opp_position[i].first,opp_position[i].second);
                double slope_of_opp2= slope(mpx,mpy,opp_position[j].first,opp_position[j].second);

                    double coeff_of_bisector_x = bisector_line_x_const(slope_of_opp1,slope_of_opp2);

                    double coeff_of_bisector_y = bisector_line_y_const(slope_of_opp1,slope_of_opp2);

                    double const_of_opp1 = constant(mpx,mpy,slope_of_opp1);
                    double const_of_opp2 = constant(mpx,mpy,slope_of_opp2);

                    double constant_of_bisector= bisector_line_const(const_of_opp1,const_of_opp2,slope_of_opp1,slope_of_opp2);

                        // find the perpendicular point on the bisector line
                        // calculate the threat
                    for(Unum l=1;l<=11;l++){

                        if((myPosition.Y() < mpAgent->GetWorldState().GetOpponent(l).GetPos().Y()) && AngleBetweenPoints(mpx,mpy,mpAgent->GetWorldState().GetOpponent(l).GetPos().X(),mpAgent->GetWorldState().GetOpponent(l).GetPos().Y(),mpAgent->GetWorldState().GetOpponent(i).GetPos().X(),mpAgent->GetWorldState().GetOpponent(i).GetPos().Y())<=90.0) {


                            double const_of_perpendicular = ((mpAgent->GetWorldState().GetOpponent(l).GetPos().X()*coeff_of_bisector_y)+(mpAgent->GetWorldState().GetOpponent(l).GetPos().Y()*coeff_of_bisector_x))/coeff_of_bisector_x; 
                            
                            double corresponding_x = intersecting_point_x(constant_of_bisector,const_of_perpendicular,coeff_of_bisector_x,coeff_of_bisector_y);
                            
                            double corresponding_y = intersecting_point_y(constant_of_bisector,const_of_perpendicular,coeff_of_bisector_x,coeff_of_bisector_y);
                            

                              double dist_mp_intersection = DistanceBetweenPoints(corresponding_x,corresponding_y,mpx,mpy);

                              double dist_op_intersection = DistanceBetweenPoints(corresponding_x,corresponding_y,mpAgent->GetWorldState().GetOpponent(l).GetPos().X(),mpAgent->GetWorldState().GetOpponent(l).GetPos().Y());

                              if(dist_mp_intersection > dist_op_intersection ){

                                    if(dist_mp_intersection < min_dist){

                                        min_dist = dist_mp_intersection;
                                        hole_point = Vector(corresponding_x,corresponding_y);
                                    }

                              }
                                      
                        }

                    }


                }

                    
                if( max_dist < min_dist){
                    max_dist = min_dist;
                    best_hole = Vector(hole_point.X(),hole_point.Y());
            }

                i=j;
            }
            
            //pick the best place
            //update occupyhole 
        }


} 

     bool PassToBestPlayer(){
        //TODO: Use transit variable for faster calling of the OccupyHole/Dasher functions
        Vector myPosition = mpAgent->GetSelf().GetPos();
        Vector currentHole = RoundToNearestHole(myPosition);

        Vector frontup = Vector(currentHole.X()+10, currentHole.Y()-10);
        Vector backup = Vector(currentHole.X()-10, currentHole.Y()-10);
        Vector frontdown = Vector(currentHole.X()+10, currentHole.Y()+10);
        Vector backdown = Vector(currentHole.X()-10, currentHole.Y()+10);

        Vector fronthor = Vector(currentHole.X()+20, currentHole.Y());
        Vector backhor = Vector(currentHole.X()-20, currentHole.Y());
        Vector upvert = Vector(currentHole.X(), currentHole.Y()-20);
        Vector downvert = Vector(currentHole.X(), currentHole.Y()+20);


        double frontup_threat =0.0;
        double backup_threat =0.0;
        double frontdown_threat =0.0;
        double backdown_threat =0.0;
        double min_threat = 10000000.0;

        double fronthor_threat = 0.0;
        double backhor_threat = 0.0;
        double upvert_threat = 0.0;
        double downvert_threat = 0.0;
        std::vector <std::pair<std::string,double> > max_threat;    
        
        double buffer = 0.5;
        double f = 0.6;


//A little change
std::cout<<"Hello I'm in PassToBestPlayer"<<std::endl;
/*for(Unum i=1; i<=11; i++){
            Vector player_pos = mpAgent->GetWorldState().GetTeammate(i).GetPos();
            if(AreSamePoints(player_pos, frontup, buffer)){
                Kicker::instance().KickBall(*mpAgent, player_pos, ServerParam::instance().ballSpeedMax()/2, KM_Hard, 0, false);
                return;
            }
        }
        for(Unum i=1; i<=11; i++){
            Vector player_pos = mpAgent->GetWorldState().GetTeammate(i).GetPos();
            if(AreSamePoints(player_pos, frontdown, buffer)){
                Kicker::instance().KickBall(*mpAgent, player_pos, ServerParam::instance().ballSpeedMax()/2, KM_Hard, 0, false);
                return;
            }
        }
        for(Unum i=1; i<=11; i++){
            Vector player_pos = mpAgent->GetWorldState().GetTeammate(i).GetPos();
            if(AreSamePoints(player_pos, backup, buffer)){
                Kicker::instance().KickBall(*mpAgent, player_pos, ServerParam::instance().ballSpeedMax()/2, KM_Hard, 0, false);
                return;
            }
        }
        for(Unum i=1; i<=11; i++){
            Vector player_pos = mpAgent->GetWorldState().GetTeammate(i).GetPos();
            if(AreSamePoints(player_pos, backdown, buffer)){
                Kicker::instance().KickBall(*mpAgent, player_pos, ServerParam::instance().ballSpeedMax()/2, KM_Hard, 0, false);
                return;
            }
        }




 */

 //     

 for(Unum i=1;i<=11;i++){
            //Vector player_opp = mpAgent->GetWorldState().mpAgent->GetWorldState().GetOpponent(i).GetPos();

            //std::cout<<"I'm in first for loop"<<std::endl;

        double ball_holder_to_opp_dis = (mpAgent->GetWorldState().GetOpponent(i).GetPos() - myPosition).Mod();
         if(ball_holder_to_opp_dis <=20.0){  

            if((myPosition.X() < mpAgent->GetWorldState().GetOpponent(i).GetPos().X()) && mpAgent->GetWorldState().GetOpponent(i).GetPos().Y() >= 0.0 ) {

            double xhole = frontup.X();
            double yhole = frontup.Y();

            std::cout<<"xhole:"<<xhole<<std::endl;

            double mpposx = myPosition.X(); 
            double mpposy = myPosition.Y();

             std::cout<<"mpposx:"<<mpposx<<std::endl;

            double slope = ((xhole - mpposx)/(yhole - mpposy));

             std::cout<<"slope:"<<slope<<std::endl;

            double ccon = yhole -(slope*xhole);

             std::cout<<"ccon:"<<ccon<<std::endl;

            double shortest_dist = abs((mpAgent->GetWorldState().GetOpponent(i).GetPos().X()*slope)+(mpAgent->GetWorldState().GetOpponent(i).GetPos().Y()*1)+(ccon))/sqrt(pow(mpAgent->GetWorldState().GetOpponent(i).GetPos().X(),2)+pow(mpAgent->GetWorldState().GetOpponent(i).GetPos().Y(),2));

            std::cout<<"shortest_dist :"<<shortest_dist<<std::endl;

            double px = xhole - mpposx;
            double py = yhole - mpposy;

            double u = (((mpAgent->GetWorldState().GetOpponent(i).GetPos().X()-mpposx)*px )+((mpAgent->GetWorldState().GetOpponent(i).GetPos().Y()-mpposy)*py))/((px*px)+(py*py));

            double currposx = mpposx + u;
            double currposy = mpposy + u;

            double dist_to_perpendicular = sqrt(pow(mpAgent->GetWorldState().GetOpponent(i).GetPos().X()-currposx,2)+pow(mpAgent->GetWorldState().GetOpponent(i).GetPos().Y()-currposy,2));

            if((dist_to_perpendicular/100.0)>(shortest_dist/100.0)) {
                frontup_threat += shortest_dist - dist_to_perpendicular; 
                 std::cout<<"frontup:"<<frontup_threat<<std::endl;
                 max_threat.push_back(std::make_pair("frontup_threat",abs(frontup_threat)));
            }
            }
        }
        }  


        for(Unum i=1;i<=11;i++){
            //Vector player_opp = mpAgent->GetWorldState().mpAgent->GetWorldState().GetOpponent(i).GetPos();

            //std::cout<<"I'm in first for loop"<<std::endl;

        double ball_holder_to_opp_dis = (mpAgent->GetWorldState().GetOpponent(i).GetPos() - myPosition).Mod();
         if(ball_holder_to_opp_dis <=20.0){  

            if((myPosition.X() > mpAgent->GetWorldState().GetOpponent(i).GetPos().X()) && mpAgent->GetWorldState().GetOpponent(i).GetPos().Y() >= 0.0 ) {

            double xhole = backup.X();
            double yhole = backup.Y();

            std::cout<<"xhole:"<<xhole<<std::endl;

            double mpposx = myPosition.X(); 
            double mpposy = myPosition.Y();

             std::cout<<"mpposx:"<<mpposx<<std::endl;

            double slope = ((xhole - mpposx)/(yhole - mpposy));

             std::cout<<"slope:"<<slope<<std::endl;

            double ccon = yhole -(slope*xhole);

             std::cout<<"ccon:"<<ccon<<std::endl;

            double shortest_dist = abs((mpAgent->GetWorldState().GetOpponent(i).GetPos().X()*slope)+(mpAgent->GetWorldState().GetOpponent(i).GetPos().Y()*1)+(ccon))/sqrt(pow(mpAgent->GetWorldState().GetOpponent(i).GetPos().X(),2)+pow(mpAgent->GetWorldState().GetOpponent(i).GetPos().Y(),2));

            std::cout<<"shortest_dist :"<<shortest_dist<<std::endl;

            double px = xhole - mpposx;
            double py = yhole - mpposy;

            double u = (((mpAgent->GetWorldState().GetOpponent(i).GetPos().X()-mpposx)*px )+((mpAgent->GetWorldState().GetOpponent(i).GetPos().Y()-mpposy)*py))/((px*px)+(py*py));

            double currposx = mpposx + u;
            double currposy = mpposy + u;

            double dist_to_perpendicular = sqrt(pow(mpAgent->GetWorldState().GetOpponent(i).GetPos().X()-currposx,2)+pow(mpAgent->GetWorldState().GetOpponent(i).GetPos().Y()-currposy,2));

            if((dist_to_perpendicular/100.0)>(shortest_dist/100.0)) {
                backup_threat += shortest_dist - dist_to_perpendicular; 
                 std::cout<<"backup:"<<backup_threat<<std::endl;
                 max_threat.push_back(std::make_pair("backup_threat",abs(backup_threat)));
            }
            }
        }
        }  

        for(Unum i=1;i<=11;i++){
            //Vector player_opp = mpAgent->GetWorldState().mpAgent->GetWorldState().GetOpponent(i).GetPos();

            //std::cout<<"I'm in first for loop"<<std::endl;

        double ball_holder_to_opp_dis = (mpAgent->GetWorldState().GetOpponent(i).GetPos() - myPosition).Mod();
         if(ball_holder_to_opp_dis <=20.0){  

            if((myPosition.X() > mpAgent->GetWorldState().GetOpponent(i).GetPos().X()) && mpAgent->GetWorldState().GetOpponent(i).GetPos().Y() <= 0.0 ) {

            double xhole = backdown.X();
            double yhole = backdown.Y();

            std::cout<<"xhole:"<<xhole<<std::endl;

            double mpposx = myPosition.X(); 
            double mpposy = myPosition.Y();

             std::cout<<"mpposx:"<<mpposx<<std::endl;

            double slope = ((xhole - mpposx)/(yhole - mpposy));

             std::cout<<"slope:"<<slope<<std::endl;

            double ccon = yhole -(slope*xhole);

             std::cout<<"ccon:"<<ccon<<std::endl;

            double shortest_dist = abs((mpAgent->GetWorldState().GetOpponent(i).GetPos().X()*slope)+(mpAgent->GetWorldState().GetOpponent(i).GetPos().Y()*1)+(ccon))/sqrt(pow(mpAgent->GetWorldState().GetOpponent(i).GetPos().X(),2)+pow(mpAgent->GetWorldState().GetOpponent(i).GetPos().Y(),2));

            std::cout<<"shortest_dist :"<<shortest_dist<<std::endl;

            double px = xhole - mpposx;
            double py = yhole - mpposy;

            double u = (((mpAgent->GetWorldState().GetOpponent(i).GetPos().X()-mpposx)*px )+((mpAgent->GetWorldState().GetOpponent(i).GetPos().Y()-mpposy)*py))/((px*px)+(py*py));

            double currposx = mpposx + u;
            double currposy = mpposy + u;

            double dist_to_perpendicular = sqrt(pow(mpAgent->GetWorldState().GetOpponent(i).GetPos().X()-currposx,2)+pow(mpAgent->GetWorldState().GetOpponent(i).GetPos().Y()-currposy,2));

            if((dist_to_perpendicular/100.0)>(shortest_dist/100.0)) {
                backdown_threat += shortest_dist - dist_to_perpendicular; 
                 std::cout<<"backdown:"<<backdown_threat<<std::endl;
                 max_threat.push_back(std::make_pair("backdown_threat",abs(backdown_threat)));
            }
            }
        }
        }  


        for(Unum i=1;i<=11;i++){
            //Vector player_opp = mpAgent->GetWorldState().mpAgent->GetWorldState().GetOpponent(i).GetPos();

            //std::cout<<"I'm in first for loop"<<std::endl;

        double ball_holder_to_opp_dis = (mpAgent->GetWorldState().GetOpponent(i).GetPos() - myPosition).Mod();
         if(ball_holder_to_opp_dis <=20.0){  

            if((myPosition.X() < mpAgent->GetWorldState().GetOpponent(i).GetPos().X()) && mpAgent->GetWorldState().GetOpponent(i).GetPos().Y() >= 0.0 ) {

            double xhole = frontdown.X();
            double yhole = frontdown.Y();

            std::cout<<"xhole:"<<xhole<<std::endl;

            double mpposx = myPosition.X(); 
            double mpposy = myPosition.Y();

             std::cout<<"mpposx:"<<mpposx<<std::endl;

            double slope = ((xhole - mpposx)/(yhole - mpposy));

             std::cout<<"slope:"<<slope<<std::endl;

            double ccon = yhole -(slope*xhole);

             std::cout<<"ccon:"<<ccon<<std::endl;

            double shortest_dist = abs((mpAgent->GetWorldState().GetOpponent(i).GetPos().X()*slope)+(mpAgent->GetWorldState().GetOpponent(i).GetPos().Y()*1)+(ccon))/sqrt(pow(mpAgent->GetWorldState().GetOpponent(i).GetPos().X(),2)+pow(mpAgent->GetWorldState().GetOpponent(i).GetPos().Y(),2));

            std::cout<<"shortest_dist :"<<shortest_dist<<std::endl;

            double px = xhole - mpposx;
            double py = yhole - mpposy;

            double u = (((mpAgent->GetWorldState().GetOpponent(i).GetPos().X()-mpposx)*px )+((mpAgent->GetWorldState().GetOpponent(i).GetPos().Y()-mpposy)*py))/((px*px)+(py*py));

            double currposx = mpposx + u;
            double currposy = mpposy + u;

            double dist_to_perpendicular = sqrt(pow(mpAgent->GetWorldState().GetOpponent(i).GetPos().X()-currposx,2)+pow(mpAgent->GetWorldState().GetOpponent(i).GetPos().Y()-currposy,2));

            if((dist_to_perpendicular/100.0)>(shortest_dist/100.0)) {
                frontdown_threat += shortest_dist - dist_to_perpendicular; 
                 std::cout<<"frontdown:"<<frontdown_threat<<std::endl;
                 max_threat.push_back(std::make_pair("frontdown_threat",abs(frontdown_threat)));
            }
            }
        }
        }  





        
        for(Unum i=1;i<=11;i++){
            //Vector player_opp = mpAgent->GetWorldState().mpAgent->GetWorldState().GetOpponent(i).GetPos();

            //std::cout<<"I'm in first for loop"<<std::endl;

        double ball_holder_to_opp_dis = (mpAgent->GetWorldState().GetOpponent(i).GetPos() - myPosition).Mod();
         if(ball_holder_to_opp_dis <=20.0){  

            if(myPosition.X() < mpAgent->GetWorldState().GetOpponent(i).GetPos().X()){

            double xhole = fronthor.X();
            double yhole = fronthor.Y();

            std::cout<<"xhole:"<<xhole<<std::endl;

            double mpposx = myPosition.X(); 
            double mpposy = myPosition.Y();

             std::cout<<"mpposx:"<<mpposx<<std::endl;

            double slope = ((xhole - mpposx)/(yhole - mpposy));

             std::cout<<"slope:"<<slope<<std::endl;

            double ccon = yhole -(slope*xhole);

             std::cout<<"ccon:"<<ccon<<std::endl;

            double shortest_dist = abs((mpAgent->GetWorldState().GetOpponent(i).GetPos().X()*slope)+(mpAgent->GetWorldState().GetOpponent(i).GetPos().Y()*1)+(ccon))/sqrt(pow(mpAgent->GetWorldState().GetOpponent(i).GetPos().X(),2)+pow(mpAgent->GetWorldState().GetOpponent(i).GetPos().Y(),2));

            std::cout<<"shortest_dist :"<<shortest_dist<<std::endl;

            double px = xhole - mpposx;
            double py = yhole - mpposy;

            double u = (((mpAgent->GetWorldState().GetOpponent(i).GetPos().X()-mpposx)*px )+((mpAgent->GetWorldState().GetOpponent(i).GetPos().Y()-mpposy)*py))/((px*px)+(py*py));

            double currposx = mpposx + u;
            double currposy = mpposy + u;

            double dist_to_perpendicular = sqrt(pow(mpAgent->GetWorldState().GetOpponent(i).GetPos().X()-currposx,2)+pow(mpAgent->GetWorldState().GetOpponent(i).GetPos().Y()-currposy,2));

            if((dist_to_perpendicular/100.0)>(shortest_dist/100.0)) {
                fronthor_threat += shortest_dist - dist_to_perpendicular; 
                 std::cout<<"fronthor:"<<fronthor_threat<<std::endl;
                 max_threat.push_back(std::make_pair("fronthor_threat",abs(fronthor_threat)));
            }
            }
        }
        }



        for(Unum i=1;i<=11;i++){
            //Vector player_opp = mpAgent->GetWorldState().mpAgent->GetWorldState().GetOpponent(i).GetPos();

            double ball_holder_to_opp_dis = (mpAgent->GetWorldState().GetOpponent(i).GetPos() - myPosition).Mod();
         if(ball_holder_to_opp_dis <=20.0){  

            if(myPosition.Y() < mpAgent->GetWorldState().GetOpponent(i).GetPos().Y()){

            double xhole = upvert.X();
            double yhole = upvert.Y();

            double mpposx = myPosition.X(); 
            double mpposy = myPosition.Y();

            double slope = ((xhole - mpposx)/(yhole - mpposy));

            double ccon = yhole -(slope*xhole);

            double shortest_dist = abs((mpAgent->GetWorldState().GetOpponent(i).GetPos().X()*slope)+(mpAgent->GetWorldState().GetOpponent(i).GetPos().Y()*1)+(ccon))/sqrt(pow(mpAgent->GetWorldState().GetOpponent(i).GetPos().X(),2)+pow(mpAgent->GetWorldState().GetOpponent(i).GetPos().Y(),2));


            double px = xhole - mpposx;
            double py = yhole - mpposy;

            double u = (((mpAgent->GetWorldState().GetOpponent(i).GetPos().X()-mpposx)*px )+((mpAgent->GetWorldState().GetOpponent(i).GetPos().Y()-mpposy)*py))/((px*px)+(py*py));

            double currposx = mpposx + u;
            double currposy = mpposy + u;

            double dist_to_perpendicular = sqrt(pow(mpAgent->GetWorldState().GetOpponent(i).GetPos().X()-currposx,2)+pow(mpAgent->GetWorldState().GetOpponent(i).GetPos().Y()-currposy,2));

            if((dist_to_perpendicular/100.0)>(shortest_dist/100.0)) {
                upvert_threat += shortest_dist - dist_to_perpendicular; 
                max_threat.push_back(std::make_pair("upvert_threat",abs(upvert_threat)));
            }
        }
        }
    }
    



        for(Unum i=1;i<=11;i++){
            //Vector player_opp = mpAgent->GetWorldState().mpAgent->GetWorldState().GetOpponent(i).GetPos();

            double ball_holder_to_opp_dis = (mpAgent->GetWorldState().GetOpponent(i).GetPos() - myPosition).Mod();
         if(ball_holder_to_opp_dis <=20.0){  

            if(myPosition.X() > mpAgent->GetWorldState().GetOpponent(i).GetPos().X()){

            double xhole = backhor.X();
            double yhole = backhor.Y();

            double mpposx = myPosition.X(); 
            double mpposy = myPosition.Y();

            double slope = ((xhole - mpposx)/(yhole - mpposy));

            double ccon = yhole -(slope*xhole);

            double shortest_dist = abs((mpAgent->GetWorldState().GetOpponent(i).GetPos().X()*slope)+(mpAgent->GetWorldState().GetOpponent(i).GetPos().Y()*1)+(ccon))/sqrt(pow(mpAgent->GetWorldState().GetOpponent(i).GetPos().X(),2)+pow(mpAgent->GetWorldState().GetOpponent(i).GetPos().Y(),2));


            double px = xhole - mpposx;
            double py = yhole - mpposy;

            double u = (((mpAgent->GetWorldState().GetOpponent(i).GetPos().X()-mpposx)*px )+((mpAgent->GetWorldState().GetOpponent(i).GetPos().Y()-mpposy)*py))/((px*px)+(py*py));

            double currposx = mpposx + u;
            double currposy = mpposy + u;

            double dist_to_perpendicular = sqrt(pow(mpAgent->GetWorldState().GetOpponent(i).GetPos().X()-currposx,2)+pow(mpAgent->GetWorldState().GetOpponent(i).GetPos().Y()-currposy,2));

            if((dist_to_perpendicular/100.0)>(shortest_dist/100.0)) {
                backhor_threat += shortest_dist - dist_to_perpendicular; 
                max_threat.push_back(std::make_pair("backhor_threat",abs(backhor_threat)));
            }
            }
        }
        }


        for(Unum i=1;i<=11;i++){
            //Vector player_opp = mpAgent->GetWorldState().mpAgent->GetWorldState().GetOpponent(i).GetPos();

            double ball_holder_to_opp_dis = (mpAgent->GetWorldState().GetOpponent(i).GetPos() - myPosition).Mod();
         if(ball_holder_to_opp_dis <=20.0){  

            if(myPosition.Y() > mpAgent->GetWorldState().GetOpponent(i).GetPos().Y()){

            double xhole = downvert.X();
            double yhole = downvert.Y();

            double mpposx = myPosition.X(); 
            double mpposy = myPosition.Y();

            double slope = ((xhole - mpposx)/(yhole - mpposy));

            double ccon = yhole -(slope*xhole);

            double shortest_dist = abs((mpAgent->GetWorldState().GetOpponent(i).GetPos().X()*slope)+(mpAgent->GetWorldState().GetOpponent(i).GetPos().Y()*1)+(ccon))/sqrt(pow(mpAgent->GetWorldState().GetOpponent(i).GetPos().X(),2)+pow(mpAgent->GetWorldState().GetOpponent(i).GetPos().Y(),2));


            double px = xhole - mpposx;
            double py = yhole - mpposy;

            double u = (((mpAgent->GetWorldState().GetOpponent(i).GetPos().X()-mpposx)*px )+((mpAgent->GetWorldState().GetOpponent(i).GetPos().Y()-mpposy)*py))/((px*px)+(py*py));

            double currposx = mpposx + u;
            double currposy = mpposy + u;

            double dist_to_perpendicular = sqrt(pow(mpAgent->GetWorldState().GetOpponent(i).GetPos().X()-currposx,2)+pow(mpAgent->GetWorldState().GetOpponent(i).GetPos().Y()-currposy,2));

            if((dist_to_perpendicular/100.0)>(shortest_dist/100.0)) {
                downvert_threat += shortest_dist - dist_to_perpendicular; 
                max_threat.push_back(std::make_pair("downvert_threat",abs(downvert_threat)));
            }
            }
        }
        }

        //MAX Threat


    std::cout<<"Calculating Threat"<<std::endl;
    std::sort (max_threat.begin(), max_threat.end());
   // std::cout<<"After sort"<<std::endl;
    char max_t ='D';
    //std::cout<<"After Max_t"<<std::endl;
    char * str="hello";
   // std::cout<<"After hello"<<std::endl;
    //std::cout<<"After const c"<<std::endl;    
    //std::cout<<"The string"<<str<<std::endl;
    double max_thr=0.0;
    char * max_thr_str = new char[50];

    

    if(!max_threat.empty()){
    //std::cout<<"Hello i'm in if conditions"<<std::endl;
        for(int i = 0; i < max_threat.size (); i++)
    {
            //std::cout<<"Hello i'm in for loop"<<std::endl;
        if(max_threat[i].second > max_thr){
            //std::cout<<"Hello i'm in second if conditions"<<std::endl;
            max_thr=max_threat[i].second;
           // std::cout<<"Hello i'm in max_thr conditions"<<max_thr<<std::endl;
           // std::cout<<"max str"<<max_threat[i].first.c_str()<<std::endl;

            /*for(int i=0;i<strlen(max_threat[i].first.c_str());i++){
                std::cout<<max_threat[i].first.c_str()[i];
            }*/
            //std::cout<<"type of str :"<<typeid(str).name()<<std::endl;
            //std::cout<<"type of str :"<<typeid(max_threat[i].first.c_str()).name()<<std::endl;
            std::strcpy(max_thr_str,max_threat[i].first.c_str());

            //std::cout<<"Hello i'm after strcpy conditions"<<std::endl;
           // const char* max_thr_str = max_threat[i].second.c_str();
        }

        std::cout << max_threat[i].first << ":" << max_threat[i].second<< std::endl; 
    }

        //std::cout<<"The max value"<<max_threat.end().first<<std::endl;
        //std::cout<<"The max string"<<max_threat.end().second<<std::endl;
       
         //strcpy(str,max_thr_str);
         //std::cout<<"After strcpy"<<std::endl;

        

        if(std::strcmp(max_thr_str,"backdown_threat")==0){
            max_t = 'A';
        }

        else if(std::strcmp(max_thr_str,"backup_threat")==0){
            max_t='B';
        }

        else if(std::strcmp(max_thr_str,"frontup_threat")==0){
            max_t='C';
        }

        else if(std::strcmp(max_thr_str,"frontdown_threat")==0){
            max_t='D';
        }


        else if(std::strcmp(max_thr_str,"fronthor_threat")==0){
            max_t='E';
        }

        else if(std::strcmp(max_thr_str,"backhor_threat")==0){
            max_t='F';
        }
        else if(std::strcmp(max_thr_str,"upvert_threat")==0){
            max_t='G';
        }

        else if(std::strcmp(max_thr_str,"downvert_threat")==0){
            max_t='H';
        }
        else{

        }

    }

    else{

            max_t = 'I';
            std::cout<<"Hello lag gayi"<<std::endl;
        }

        switch(max_t){

            case 'A':
                        std::cout<<"In case A"<<std::endl;
                        for(Unum i=1;i<=11;i++){
                            Vector player_pos = mpAgent->GetWorldState().GetTeammate(i).GetPos();
                            if(AreSamePoints(player_pos, backdown, buffer)){
                                std::string msgstr = "cusp";
                                Unum mynum = mpAgent->GetSelfUnum();
                                std::string result;
                                std::stringstream sstm;
                                sstm << msgstr << mynum <<"X"<< i;
                                result.append(sstm.str());
                                std::cout<<"saying - "<<result<<std::endl;
                                while(!(mpAgent->Say(result)));
                                    std::cout<<"I kicked towards backdown"<<std::endl;
                                    return Kicker::instance().KickBall(*mpAgent, backdown, ServerParam::instance().ballSpeedMax()*f, KM_Hard, 0, false);   
                            }
                            else{
                                continue;
                            }

                        }
                                
            case 'B':
                        std::cout<<"In case B"<<std::endl;
                        for(Unum i=1;i<=11;i++){
                            Vector player_pos = mpAgent->GetWorldState().GetTeammate(i).GetPos();
                                if(AreSamePoints(player_pos,backup,buffer)){
                                    std::string msgstr = "cusp";
                                Unum mynum = mpAgent->GetSelfUnum();
                                std::string result;
                                std::stringstream sstm;
                                sstm << msgstr << mynum <<"X"<< i;
                                result.append(sstm.str());
                                std::cout<<"saying - "<<result<<std::endl;
                                while(!(mpAgent->Say(result)));
                                std::cout<<"I kicked towards backup"<<std::endl;
                                return Kicker::instance().KickBall(*mpAgent, backup, ServerParam::instance().ballSpeedMax()*f, KM_Hard, 0, false);   
                        
                            }
                                else{
                                    continue;
                                }

                        }
                                
            case 'C':
                    std::cout<<"In case C"<<std::endl;
                        for(Unum i=1;i<=11;i++){
                            Vector player_pos = mpAgent->GetWorldState().GetTeammate(i).GetPos();
                            if(AreSamePoints(player_pos,frontup,buffer)){
                                std::string msgstr = "cusp";
                                Unum mynum = mpAgent->GetSelfUnum();
                                std::string result;
                                std::stringstream sstm;
                                sstm << msgstr << mynum <<"X"<< i;
                                result.append(sstm.str());
                                std::cout<<"saying - "<<result<<std::endl;
                                while(!(mpAgent->Say(result)));
                                std::cout<<"I kicked towards frontup"<<std::endl;
                                return Kicker::instance().KickBall(*mpAgent, frontup, ServerParam::instance().ballSpeedMax()*f, KM_Hard, 0, false);
                                
                            }
                            else{
                                continue;
                            }

                        }
                                
            case 'D':
                    std::cout<<"In case D"<<std::endl;
                        for(Unum i=1;i<=11;i++){
                            Vector player_pos = mpAgent->GetWorldState().GetTeammate(i).GetPos();
                            if(AreSamePoints(player_pos,frontdown,buffer)){
                                std::string msgstr = "cusp";
                                Unum mynum = mpAgent->GetSelfUnum();
                                std::string result;
                                std::stringstream sstm;
                                sstm << msgstr << mynum <<"X"<< i;
                                result.append(sstm.str());
                                std::cout<<"saying - "<<result<<std::endl;
                                while(!(mpAgent->Say(result)));
                                std::cout<<"I kicked towards frontdown"<<std::endl;
                                return Kicker::instance().KickBall(*mpAgent, frontdown, ServerParam::instance().ballSpeedMax()*f, KM_Hard, 0, false);
                                
                            }
                            else{
                               continue;
                            }

                        }
                        break;

            case 'E':
                        std::cout<<"In case E"<<std::endl;

                        for(Unum i=1;i<=11;i++){
                            Vector player_pos = mpAgent->GetWorldState().GetTeammate(i).GetPos();
                            if(AreSamePoints(player_pos,fronthor,buffer)){
                                std::string msgstr = "cusp";
                                Unum mynum = mpAgent->GetSelfUnum();
                                std::string result;
                                std::stringstream sstm;
                                sstm << msgstr << mynum <<"X"<< i;
                                result.append(sstm.str());
                                std::cout<<"saying - "<<result<<std::endl;
                                while(!(mpAgent->Say(result)));
                                std::cout<<"I kicked towards fronthor"<<std::endl;
                                return Kicker::instance().KickBall(*mpAgent, fronthor, ServerParam::instance().ballSpeedMax()*f, KM_Hard, 0, false);
                            
                            }
                            else{
                               continue;
                            }

                        }
                        break;

                case 'F':

                        std::cout<<"In case F"<<std::endl;
                        for(Unum i=1;i<=11;i++){
                            Vector player_pos = mpAgent->GetWorldState().GetTeammate(i).GetPos();
                            if(AreSamePoints(player_pos,backhor,buffer)){
                                std::string msgstr = "cusp";
                                Unum mynum = mpAgent->GetSelfUnum();
                                std::string result;
                                std::stringstream sstm;
                                sstm << msgstr << mynum <<"X"<< i;
                                result.append(sstm.str());
                                std::cout<<"saying - "<<result<<std::endl;
                                while(!(mpAgent->Say(result)));
                                std::cout<<"I kicked towards backhor"<<std::endl;
                                return Kicker::instance().KickBall(*mpAgent, backhor, ServerParam::instance().ballSpeedMax()*f, KM_Hard, 0, false);
                                
                            }
                            else{
                               continue;
                            }
                        }
                        break;

                case 'G':
                        std::cout<<"In case G"<<std::endl;
                        for(Unum i=1;i<=11;i++){
                            Vector player_pos = mpAgent->GetWorldState().GetTeammate(i).GetPos();
                            if(AreSamePoints(player_pos,upvert,buffer)){
                                std::string msgstr = "cusp";
                                Unum mynum = mpAgent->GetSelfUnum();
                                std::string result;
                                std::stringstream sstm;
                                sstm << msgstr << mynum <<"X"<< i;
                                result.append(sstm.str());
                                std::cout<<"saying - "<<result<<std::endl;
                                while(!(mpAgent->Say(result)));
                                std::cout<<"I kicked towards upvert"<<std::endl;
                                return Kicker::instance().KickBall(*mpAgent, upvert, ServerParam::instance().ballSpeedMax()*f, KM_Hard, 0, false);
                                
                            }
                            else{
                               continue;
                            }

                        }
                        break;

                case 'H':
                        std::cout<<"In case H"<<std::endl;
                        for(Unum i=1;i<=11;i++){
                            Vector player_pos = mpAgent->GetWorldState().GetTeammate(i).GetPos();
                            if(AreSamePoints(player_pos,downvert,buffer)){
                                std::string msgstr = "cusp";
                                Unum mynum = mpAgent->GetSelfUnum();
                                std::string result;
                                std::stringstream sstm;
                                sstm << msgstr << mynum <<"X"<< i;
                                result.append(sstm.str());
                                std::cout<<"saying - "<<result<<std::endl;
                                while(!(mpAgent->Say(result)));
                                std::cout<<"I kicked towards downvert"<<std::endl;
                                return Kicker::instance().KickBall(*mpAgent, downvert, ServerParam::instance().ballSpeedMax()*f, KM_Hard, 0, false);
                                
                            }
                            else{
                               continue;
                            }

                        }
                        break;
                case 'I':
                       std::cout<<"In case I"<<std::endl;
                       for(Unum i=1;i<=11;i++){

                            Vector player_pos = mpAgent->GetWorldState().GetTeammate(i).GetPos();
                            if(AreSamePoints(player_pos,fronthor,buffer)){
                                std::string msgstr = "cusp";
                                Unum mynum = mpAgent->GetSelfUnum();
                                std::string result;
                                std::stringstream sstm;
                                sstm << msgstr << mynum <<"X"<< i;
                                result.append(sstm.str());
                                std::cout<<"saying - "<<result<<std::endl;
                                while(!(mpAgent->Say(result)));
                                std::cout<<"passing to fronthor"<<std::endl;
                                return Kicker::instance().KickBall(*mpAgent, fronthor, ServerParam::instance().ballSpeedMax()*f, KM_Hard, 0, false);
                            }
                            else if(AreSamePoints(player_pos,frontup,buffer)){
                                std::string msgstr = "cusp";
                                Unum mynum = mpAgent->GetSelfUnum();
                                std::string result;
                                std::stringstream sstm;
                                sstm << msgstr << mynum <<"X"<< i;
                                result.append(sstm.str());
                                std::cout<<"saying - "<<result<<std::endl;
                                while(!(mpAgent->Say(result)));
                                std::cout<<"passing to frontup"<<std::endl;
                                return Kicker::instance().KickBall(*mpAgent, frontup, ServerParam::instance().ballSpeedMax()*f, KM_Hard, 0, false);
                                
                             }
                            else if(AreSamePoints(player_pos,frontdown,buffer)){
                                std::string msgstr = "cusp";
                                Unum mynum = mpAgent->GetSelfUnum();
                                std::string result;
                                std::stringstream sstm;
                                sstm << msgstr << mynum <<"X"<< i;
                                result.append(sstm.str());
                                std::cout<<"saying - "<<result<<std::endl;
                                while(!(mpAgent->Say(result)));
                                std::cout<<"passing to frontdown"<<std::endl;
                                return Kicker::instance().KickBall(*mpAgent, frontdown, ServerParam::instance().ballSpeedMax()*f, KM_Hard, 0, false);
                            }

                            else if(AreSamePoints(player_pos,backdown,buffer)){
                                std::string msgstr = "cusp";
                                Unum mynum = mpAgent->GetSelfUnum();
                                std::string result;
                                std::stringstream sstm;
                                sstm << msgstr << mynum <<"X"<< i;
                                result.append(sstm.str());
                                std::cout<<"saying - "<<result<<std::endl;
                                while(!(mpAgent->Say(result)));
                                std::cout<<"passing to backdown"<<std::endl;
                                return Kicker::instance().KickBall(*mpAgent, backdown, ServerParam::instance().ballSpeedMax()*f, KM_Hard, 0, false);
                            }
                            else if(AreSamePoints(player_pos,backup,buffer)){
                                std::string msgstr = "cusp";
                                Unum mynum = mpAgent->GetSelfUnum();
                                std::string result;
                                std::stringstream sstm;
                                sstm << msgstr << mynum <<"X"<< i;
                                result.append(sstm.str());
                                std::cout<<"saying - "<<result<<std::endl;
                                while(!(mpAgent->Say(result)));
                                std::cout<<"passing to backup"<<std::endl;
                                return Kicker::instance().KickBall(*mpAgent, backup, ServerParam::instance().ballSpeedMax()*f, KM_Hard, 0, false);
                            }
                            else if(AreSamePoints(player_pos,backhor,buffer)){
                                std::string msgstr = "cusp";
                                Unum mynum = mpAgent->GetSelfUnum();
                                std::string result;
                                std::stringstream sstm;
                                sstm << msgstr << mynum <<"X"<< i;
                                result.append(sstm.str());
                                std::cout<<"saying - "<<result<<std::endl;
                                while(!(mpAgent->Say(result)));
                                std::cout<<"passing to backhor"<<std::endl;
                                return Kicker::instance().KickBall(*mpAgent, backhor, ServerParam::instance().ballSpeedMax()*f, KM_Hard, 0, false);
                            }

                            else if(AreSamePoints(player_pos,upvert,buffer)){
                                std::string msgstr = "cusp";
                                Unum mynum = mpAgent->GetSelfUnum();
                                std::string result;
                                std::stringstream sstm;
                                sstm << msgstr << mynum <<"X"<< i;
                                result.append(sstm.str());
                                std::cout<<"saying - "<<result<<std::endl;
                                while(!(mpAgent->Say(result)));
                                std::cout<<"passing to upvert"<<std::endl;
                                return Kicker::instance().KickBall(*mpAgent, upvert, ServerParam::instance().ballSpeedMax()*f, KM_Hard, 0, false);
                            }
                            else{
                                continue;
                            }
                                
                       } 
                       break;
                            
                                
            default:
                    std::cout<<"You can reach here"<<std::endl;
                    return false;
        }
        return false;
    }

    bool IsOccupied(Vector target, double buffer){
    	//Returns true if target is occupied by a player
    	for(Unum i=4; i<=11; i++){
    		Vector player_pos = mpAgent->GetWorldState().GetTeammate(i).GetPos();
    		if(AreSamePoints(player_pos, target, buffer)&&i!=mpAgent->GetSelfUnum())
    			return true;
    	}
    	return false;
    }

    Unum GetOccupierUnum(Vector target, double buffer){
        for(Unum i=4; i<=11; i++){
            Vector player_pos = mpAgent->GetWorldState().GetTeammate(i).GetPos();
            if(AreSamePoints(player_pos, target, buffer))
                return i;
        }
        return -1;
    }

    Unum GetClosest(Vector target){
    	//Excluded ballholder from the below equation
    	double mindis = 99999;
    	Unum mindisUnum = 99;
    	Unum BHUnum = GetBHUnum();
    	if(BHUnum==-1){
    		std::cout<<"No ball holder"<<std::endl;
            return -1;
        }
    	for(Unum i=4; i<=11; i++){
    		double PlayerDis = mpAgent->GetWorldState().GetTeammate(i).GetPos().Dist(target);
    		if(PlayerDis<=mindis&&i!=BHUnum){
    			mindis = PlayerDis;
    			mindisUnum = i;
    		}
    	}

        return mindisUnum;
    	//std::cout<<"Player "<<mpAgent->GetSelfUnum()<<"thinks player "<<mindisUnum<<"is closest to hole"<<std::endl;
    	/*
        if(mpAgent->GetSelfUnum()==mindisUnum)
    		return true;
    	else 
    		return false;
            */
    }

    Unum GetClosestExcl1(Vector target, Unum ex1){
        //Excluded ballholder and ex1 from the below equation
        //ex1 would already have been assigned to another hole
        double mindis = 999;
        Unum mindisUnum = 99;
        Unum BHUnum = GetBHUnum();
        if(BHUnum==-1)
            std::cout<<"No ball holder"<<std::endl;
        for(Unum i=4; i<=11; i++){
            double PlayerDis = mpAgent->GetWorldState().GetTeammate(i).GetPos().Dist(target);
            if(PlayerDis<=mindis&&i!=BHUnum&&i!=ex1){
                mindis = PlayerDis;
                mindisUnum = i;
            }
        }
        return mindisUnum;
        //std::cout<<"Player "<<mpAgent->GetSelfUnum()<<"thinks player "<<mindisUnum<<"is closest to hole"<<std::endl;
        /*
        if(mpAgent->GetSelfUnum()==mindisUnum)
            return true;
        else 
            return false;
            */
    }

    Unum GetClosestExcl2(Vector target, Unum ex1, Unum ex2){
        //Excluded ballholder and ex1,2 from the below equation
        //ex1,2 would already have been assigned to another hole
        double mindis = 999;
        Unum mindisUnum = 99;
        Unum BHUnum = GetBHUnum();
        if(BHUnum==-1)
            std::cout<<"No ball holder"<<std::endl;
        for(Unum i=4; i<=11; i++){
            double PlayerDis = mpAgent->GetWorldState().GetTeammate(i).GetPos().Dist(target);
            if(PlayerDis<=mindis&&i!=BHUnum&&i!=ex1&&i!=ex2){
                mindis = PlayerDis;
                mindisUnum = i;
            }
        }
       return mindisUnum;
        //std::cout<<"Player "<<mpAgent->GetSelfUnum()<<"thinks player "<<mindisUnum<<"is closest to hole"<<std::endl;
        /*
        if(mpAgent->GetSelfUnum()==mindisUnum)
            return true;
        else 
            return false;
            */
    }

    Unum GetClosestExcl3(Vector target, Unum ex1, Unum ex2, Unum ex3){
        //Excluded ballholder and ex1,2,3 from the below equation
        //ex1,2,3 would already have been assigned to another hole
        double mindis = 999;
        Unum mindisUnum = 99;
        Unum BHUnum = GetBHUnum();
        if(BHUnum==-1)
            std::cout<<"No ball holder"<<std::endl;
        for(Unum i=4; i<=11; i++){
            double PlayerDis = mpAgent->GetWorldState().GetTeammate(i).GetPos().Dist(target);
            if(PlayerDis<=mindis&&i!=BHUnum&&i!=ex1&&i!=ex2&&i!=ex3){
                mindis = PlayerDis;
                mindisUnum = i;
            }
        }
        return mindisUnum;
        //std::cout<<"Player "<<mpAgent->GetSelfUnum()<<"thinks player "<<mindisUnum<<"is closest to hole"<<std::endl;
        /*
        if(mpAgent->GetSelfUnum()==mindisUnum)
            return true;
        else 
            return false;
            */
    }

    Unum GetClosestExcl4(Vector target, Unum ex1, Unum ex2, Unum ex3, Unum ex4){
        double mindis = 999;
        Unum mindisUnum = 99;
        Unum BHUnum = GetBHUnum();
        if(BHUnum==-1)
            std::cout<<"No ball holder"<<std::endl;
        for(Unum i=4; i<=11; i++){
            double PlayerDis = mpAgent->GetWorldState().GetTeammate(i).GetPos().Dist(target);
            if(PlayerDis<=mindis&&i!=BHUnum&&i!=ex1&&i!=ex2&&i!=ex3&&i!=ex4){
                mindis = PlayerDis;
                mindisUnum = i;
            }
        }
        return mindisUnum;
    }

    Unum GetClosestExcl5(Vector target, Unum ex1, Unum ex2, Unum ex3, Unum ex4, Unum ex5){
        double mindis = 999;
        Unum mindisUnum = 99;
        Unum BHUnum = GetBHUnum();
        if(BHUnum==-1)
            std::cout<<"No ball holder"<<std::endl;
        for(Unum i=4; i<=11; i++){
            double PlayerDis = mpAgent->GetWorldState().GetTeammate(i).GetPos().Dist(target);
            if(PlayerDis<=mindis&&i!=BHUnum&&i!=ex1&&i!=ex2&&i!=ex3&&i!=ex4&&i!=ex5){
                mindis = PlayerDis;
                mindisUnum = i;
            }
        }
        return mindisUnum;
    }

    Unum GetClosestExcl6(Vector target, Unum ex1, Unum ex2, Unum ex3, Unum ex4, Unum ex5, Unum ex6){
        double mindis = 999;
        Unum mindisUnum = 99;
        Unum BHUnum = GetBHUnum();
        if(BHUnum==-1)
            std::cout<<"No ball holder"<<std::endl;
        for(Unum i=4; i<=11; i++){
            double PlayerDis = mpAgent->GetWorldState().GetTeammate(i).GetPos().Dist(target);
            if(PlayerDis<=mindis&&i!=BHUnum&&i!=ex1&&i!=ex2&&i!=ex3&&i!=ex4&&i!=ex5&&i!=ex6){
                mindis = PlayerDis;
                mindisUnum = i;
            }
        }
        return mindisUnum;
    }

    Unum GetClosestExcl7(Vector target, Unum ex1, Unum ex2, Unum ex3, Unum ex4, Unum ex5, Unum ex6, Unum ex7){
        double mindis = 999;
        Unum mindisUnum = 99;
        Unum BHUnum = GetBHUnum();
        if(BHUnum==-1)
            std::cout<<"No ball holder"<<std::endl;
        for(Unum i=4; i<=11; i++){
            double PlayerDis = mpAgent->GetWorldState().GetTeammate(i).GetPos().Dist(target);
            if(PlayerDis<=mindis&&i!=BHUnum&&i!=ex1&&i!=ex2&&i!=ex3&&i!=ex4&&i!=ex5&&i!=ex6&&i!=ex7){
                mindis = PlayerDis;
                mindisUnum = i;
            }
        }
        return mindisUnum;
    }

    Unum GetClosestTeammateTo(Vector target){
        double mindis = 99999;
        Unum mindisUnum = -1;
        for(Unum i=4; i<=11; i++){
            Vector player_pos = mpAgent->GetWorldState().GetTeammate(i).GetPos();
            double dis = player_pos.Dist(target);
            if(dis<mindis){
                mindisUnum = i;
                mindis=dis;
            }
        }
        return mindisUnum;
    }

    Unum GetClosestFromArray(Vector target){
        double mindis = 99999;
        Unum mindisUnum = -1;
        for(int i=0; i<playervector.size(); i++){
            if(playervector[i]!=-1){
                Vector player_pos = mpAgent->GetWorldState().GetTeammate(playervector[i]).GetPos();
                double dis = player_pos.Dist(target);
                if(dis<mindis){
                    mindisUnum = playervector[i];
                    mindis=dis;
                }
            }
        }
        return mindisUnum;
    }

    void DecideAndOccupyHole(Unum target){
        //Called when another teammate would have the ball
        //target == -1 means a player already has the ball
        //target == -2 means ballpos+somevector assumed as ballholder pos
        //target == -3 means player has the ball, and the current player couldnt occupy a hole
        IsOccupying = false;
        
        Vector BHPos;
                
        if(target==-1){
            Unum i=1;
            for(i=4; i<=11; i++){
                if(mpAgent->GetWorldState().GetTeammate(i).IsKickable()&&(i!=mpAgent->GetSelfUnum())){
                    BHPos = mpAgent->GetWorldState().GetTeammate(i).GetPos();
                    break;
                }
            }
            if(i==12)
                return;
        }
        /*
        else if(target==-2){
            BHPos = mpAgent->GetWorldState().GetBall().GetPos();
            BHPos = Vector(BHPos.X()+40,BHPos.Y());
        }
        else if(target ==-3){
             for(Unum i=1; i<=11; i++){
                if(mpAgent->GetWorldState().GetTeammate(i).IsKickable()&&(i!=mpAgent->GetSelfUnum())){
                    BHPos = mpAgent->GetWorldState().GetTeammate(i).GetPos();
                    BHPos = Vector(BHPos.X()+20, BHPos.Y());
                    break;
                }
            }
        }
        */
        else if(target!=mpAgent->GetSelfUnum()){
            BHPos = mpAgent->GetWorldState().GetTeammate(target).GetPos();
        }
        else
            return;
        
        BHPos = RoundToNearestHole(BHPos);

        Vector BHfrontup = Vector(BHPos.X()+10, BHPos.Y()-10);
        Vector BHbackup = Vector(BHPos.X()-10, BHPos.Y()-10);
        Vector BHfrontdown = Vector(BHPos.X()+10, BHPos.Y()+10);
        Vector BHbackdown = Vector(BHPos.X()-10, BHPos.Y()+10);

        Vector BHfronthor = Vector(BHPos.X()+20, BHPos.Y());
        Vector BHbackhor = Vector(BHPos.X()-20, BHPos.Y());
        Vector BHupvert = Vector(BHPos.X(), BHPos.Y()-20);
        Vector BHdownvert = Vector(BHPos.X(), BHPos.Y()+20);

        ////////////////////////////////////////////////////////////
      /*  holevector.clear();
        playervector.clear();
        holemap.clear();
        playermap.clear();

        holevector.push_back(BHfrontup);
        holevector.push_back(BHfronthor);
        holevector.push_back(BHfrontdown);
        holevector.push_back(BHupvert);
        holevector.push_back(BHdownvert);
        holevector.push_back(BHbackup);
        holevector.push_back(BHbackdown);
        holevector.push_back(BHbackhor);

        //holevector[0] = Vector(100,100);
        //holevector[1] = Vector(100,100);
        //holevector[2] = Vector(100,100);
        holevector[3] = Vector(100,100);
        holevector[4] = Vector(100,100);
        //holevector[5] = Vector(100,100);
        //holevector[6] = Vector(100,100);
        holevector[7] = Vector(100,100);
            
        for(Unum i=4; i<=11; i++)
            playervector.push_back(i);

        Vector mypos = mpAgent->GetSelf().GetPos();
        double buffer = 1;
        Vector test;
        for(int i=0; i<holevector.size(); i++){
            if(holevector[i]!=Vector(100,100)){
                test = holevector[i];
                Unum X = GetOccupierUnum(test, buffer); //X will be the smaller unum if two players occupy a hole
                if(X==mpAgent->GetSelfUnum() && test.X()>=-50 && test.X()<=50 && test.Y()>=-25 && test.Y()<=25){
                    OccupyHole(test);
                    return;
                }
            }
        }

        for(int i=0; i<holevector.size(); i++){
            if(holevector[i]!=Vector(100,100)){
                Unum occupier = GetOccupierUnum(holevector[i], buffer);
                if(occupier!=-1){
                    playermap[occupier] = holevector[i];
                    holemap[holevector[i]] = occupier; 
                }
            }
        }

        std::map<Vector,int>::iterator holeit;
        std::map<int,Vector>::iterator playit;
        
        for(int i=0; i<holevector.size(); i++){
            if(holevector[i]!=Vector(100,100)){
                holeit = holemap.find(holevector[i]);
                if(holeit!=holemap.end()){
                    std::cout<<"Found hole in map"<<std::endl;
                    holevector[i] = Vector(100, 100);
                }
            }
        }

        for(int i=0; i<playervector.size(); i++){
            if(playervector[i]!=-1){
                playit = playermap.find(playervector[i]);
                if(playit!=playermap.end()){
                    std::cout<<"Found player in map"<<std::endl;
                    playervector[i] = -1;
                }
            }
        }

        for(int i=0; i<holevector.size(); i++){
            if(holevector[i]!=Vector(100,100)){
                Unum X = GetClosestFromArray(holevector[i]);
                if(X==mpAgent->GetSelfUnum() && holevector[i].X()>=-50 && holevector[i].X()<=50 && holevector[i].Y()>=-25 && holevector[i].Y()<=25){
                    OccupyHole(holevector[i]);
                    return;
                }
            }
        }*/
        //////////////////////////////////////////////////////////////////

        
        Unum FU, FD, BU, BD;
        Unum FH, BH, UV, DV;

        //&&BHfrontup.X()>=-45&&BHfrontup.X()<=45&&BHfrontup.Y()>=-25&&BHfrontup.Y()<=25

        FU = GetClosest(BHfrontup);
       // std::cout<<"For player "<<mpAgent->GetSelfUnum()<<", FU = "<<FU<<std::endl;
        if(FU==mpAgent->GetSelfUnum())
            //std::cout<<"might be eligible - FU - "<<BHfrontup<<std::endl;
        if(FU==mpAgent->GetSelfUnum()&&BHfrontup.X()>=-50&&BHfrontup.X()<=50&&BHfrontup.Y()>=-25&&BHfrontup.Y()<=25){
            OccupyHole(BHfrontup);
            //std::cout<<"Player "<<FU<<" occupying frontup"<<std::endl;
            return;
        }

        FD = GetClosestExcl1(BHfrontdown, FU);
        if(FD==mpAgent->GetSelfUnum())
            //std::cout<<"might be eligible - FD - "<<BHfrontdown<<std::endl;
        //std::cout<<"For player "<<mpAgent->GetSelfUnum()<<", FD = "<<FD<<std::endl;
        if(FD==mpAgent->GetSelfUnum()&&BHfrontdown.X()>=-50&&BHfrontdown.X()<=50&&BHfrontdown.Y()>=-25&&BHfrontdown.Y()<=25){
            OccupyHole(BHfrontdown);
            //std::cout<<"Player "<<FD<<" occupying frontdown"<<std::endl;
            return;
        }

        FH = GetClosestExcl2(BHfronthor, FU, FD);
        if(FH==mpAgent->GetSelfUnum())
           // std::cout<<"might be eligible - FH - "<<BHfronthor<<std::endl;
        //std::cout<<"For player "<<mpAgent->GetSelfUnum()<<", FH = "<<FH<<std::endl;
        if(FH==mpAgent->GetSelfUnum()&&BHfronthor.X()>=-50&&BHfronthor.X()<=50&&BHfronthor.Y()>=-25&&BHfronthor.Y()<=25){
            OccupyHole(BHfronthor);
           // std::cout<<"Player "<<FH<<" occupying fronthor"<<std::endl;
            return;
        }

        UV = GetClosestExcl3(BHupvert, FU, FD, FH);
        if(UV==mpAgent->GetSelfUnum())
           // std::cout<<"might be eligible - UV - "<<BHupvert<<std::endl;
        //std::cout<<"For player "<<mpAgent->GetSelfUnum()<<", UV = "<<UV<<std::endl;
        if(UV==mpAgent->GetSelfUnum()&&BHupvert.X()>=-50&&BHupvert.X()<=50&&BHupvert.Y()>=-25&&BHupvert.Y()<=25){
            OccupyHole(BHupvert);
           // std::cout<<"Player "<<UV<<" occupying upvert"<<std::endl;
            return;
        }

        DV = GetClosestExcl4(BHdownvert, FU, FD, FH, UV);
        if(DV==mpAgent->GetSelfUnum())
           // std::cout<<"might be eligible - DV - "<<BHdownvert<<std::endl;
        //std::cout<<"For player "<<mpAgent->GetSelfUnum()<<", DV = "<<DV<<std::endl;
        if(DV==mpAgent->GetSelfUnum()&&BHdownvert.X()>=-50&&BHdownvert.X()<=50&&BHdownvert.Y()>=-25&&BHdownvert.Y()<=25){
            OccupyHole(BHdownvert);
          //  std::cout<<"Player "<<DV<<" occupying downvert"<<std::endl;
            return;
        }

        BU = GetClosestExcl5(BHbackup, FU, FD, FH, UV, DV);
        if(BU==mpAgent->GetSelfUnum())
           // std::cout<<"might be eligible - BU - "<<BHbackup<<std::endl;
        //std::cout<<"For player "<<mpAgent->GetSelfUnum()<<", BU = "<<BU<<std::endl;
        if(BU==mpAgent->GetSelfUnum()&&BHbackup.X()>=-50&&BHbackup.X()<=50&&BHbackup.Y()>=-25&&BHbackup.Y()<=25){
            OccupyHole(BHbackup);
           // std::cout<<"Player "<<BU<<" occupying backup"<<std::endl;
            return;
        }
        
        BD = GetClosestExcl6(BHbackdown, FU, FD, FH, UV, DV, BU);
        if(BD==mpAgent->GetSelfUnum())
            //std::cout<<"might be eligible - BD - "<<BHbackdown<<std::endl;
        //std::cout<<"For player "<<mpAgent->GetSelfUnum()<<", BD = "<<BD<<std::endl;
        if(BD==mpAgent->GetSelfUnum()&&BHbackdown.X()>=-50&&BHbackdown.X()<=50&&BHbackdown.Y()>=-25&&BHbackdown.Y()<=25){
            OccupyHole(BHbackdown);
           // std::cout<<"Player "<<BD<<" occupying backdown"<<std::endl;
            return;
        }

        //BH = GetClosestExcl7(BHbackhor, FU, FD, FH, UV, DV, BU, BD);
        BH = mpAgent->GetSelfUnum();
        if(BH==mpAgent->GetSelfUnum())
           // std::cout<<"might be eligible - BH - "<<BHbackhor<<std::endl;
       // std::cout<<"For player "<<mpAgent->GetSelfUnum()<<", BH = "<<BH<<std::endl;
        if(BH==mpAgent->GetSelfUnum()&&BHbackhor.X()>=-50&&BHbackhor.X()<=50&&BHbackhor.Y()>=-25&&BHbackhor.Y()<=25){
            OccupyHole(BHbackhor);
           // std::cout<<"Player "<<BH<<" occupying backhor"<<std::endl;
            return;
        }
        
        
        /*
        std::vector<Vector> holes;
        holes.push_back(BHfrontup);
        holes.push_back(BHbackup);
        holes.push_back(BHfrontdown);
        holes.push_back(BHbackdown);
        holes.push_back(BHfronthor);
        holes.push_back(BHbackhor);
        holes.push_back(BHupvert);
        holes.push_back(BHdownvert);

        Vector targethole;
        double mindis = 999999;
        Vector myPos = mpAgent->GetSelf().GetPos();

        for(int i=0; i<8; i++){
            double dis = myPos.Dist(holes[i]);
            if(dis<mindis){
                mindis = dis;
                targethole = holes[i];
            }
        }

        OccupyHole(targethole);
        */

        //if frontup&frontdown not occupied (+ other conditions), move there
        //TODO: Currently, it is possible that one player is expected to fill both the holes

        /*
        
        */
    }

    bool BallKickableByATeammate(){
    	//TODO: Replace IsKickable with AreSamePoints + larger buffer
    	for(Unum i=1; i<=11; i++){
    		if(mpAgent->GetWorldState().GetTeammate(i).IsKickable()&&(i!=mpAgent->GetSelfUnum())){
    			//std::cout<<"Player "<<mpAgent->GetSelfUnum()<<" thinks ball kickable by Player "<<i<<std::endl;
    			return true;
    		}
    	}
    	return false;
    }

    Unum GetBHUnum(){
    	//Only to be called if BallKickableByATeammate
        if(ResetCallOccupy)
            return 100; //Return passer unum
    	for(Unum i=1; i<=11; i++){
    		if(mpAgent->GetWorldState().GetTeammate(i).IsKickable())
    			return i;
    	}
    	return GetClosestUnumToBall();
    }

    Unum GetClosestUnumToBall(){
        Vector ballpos = mpAgent->GetWorldState().GetBall().GetPos();
        Unum mindisUnum = 0;
        double mindis = 9999;
        for(Unum i=1; i<=11; i++){
            Vector UnumPos = mpAgent->GetWorldState().GetTeammate(i).GetPos();
            double dis = UnumPos.Dist(ballpos);
            if(dis<mindis){
                mindis = dis;
                mindisUnum = i;
            }
        }
        return mindisUnum;
    }


	bool AreSamePoints(Vector A, Vector B, double buffer){
		//Check if and b are the same points +/- buffer
		if( (abs(A.X()-B.X())<buffer) && (abs(A.Y()-B.Y())<buffer))
    		return true;
    	else
    		return false;
	}

	void OccupyHole(Vector target){
	    //Dash to target
	    //while(!AreSamePoints(mpAgent->GetSelf().GetPos(),target,2))
		    //Dasher::instance().GoToPoint(*mpAgent, target, 0.3, 100, true, false);
        mpIntransit = true;
        mpTarget = target;
        IsOccupying = true;
	}

	double abs(double d){
	    if (d>0.00)
	            return d;
	        else
	            return d*(-1.00);
	}

	Vector RoundToNearestTens(Vector P){
	    // This method rounds a given position to its nearest tens - for example, the rounded position for (12, -17) would be (10, -20)
	    // This helps in locating nearby holes more easily
	    double multX = 10.00;
	    double multY = 10.00;
	    if(P.X()<0.00)
	        multX = -10.00;
	    if(P.Y()<0.00)
	        multY = -10.00;
	    int roundX = static_cast<int>((abs(P.X())+5.00)/10);
	    int roundY = static_cast<int>((abs(P.Y())+5.00)/10);
	    Vector roundedTens = Vector(multX*roundX, multY*roundY);
	    //std::cout<<"Rounded Tens - "<<roundedTens<<std::endl;
	    return roundedTens;
	}

	bool isRTaHole(Vector P){
	    // This method is only for rounded tens
	    // Returns true iff rounded-ten point is a hole    
	    int normalX = static_cast<int>(abs(P.X())/10);
	    int normalY = static_cast<int>(abs(P.Y())/10);
	    if(normalX%2==normalY%2)
	        return true;
	    else
	        return false;
	}

	Vector RoundToNearestHole(Vector P){
	    //std::cout<<"Rounding up point - "<<P<<std::endl;
	    Vector roundedTens = RoundToNearestTens(P);
	    if(isRTaHole(roundedTens)){
	        //std::cout<<"RT is a hole - "<<roundedTens<<std::endl;
	        return roundedTens;
	    }
	    else{
	        Vector roundedHole;
	        double diffX = P.X()-roundedTens.X();
	        double diffY = P.Y()-roundedTens.Y();
	            
	        if(abs(diffX)<abs(diffY)){
	            //Point closer to vertical axis of the diamond
	            if(diffY>0)
	                roundedHole = Vector(roundedTens.X(), roundedTens.Y()+10);
	            else
	                roundedHole = Vector(roundedTens.X(), roundedTens.Y()-10);
	        }
	        else{
	            //Point closer to horizontal axis of the diamond
	            if(diffX>0)
	                roundedHole = Vector(roundedTens.X()+10, roundedTens.Y());
	            else
	                roundedHole = Vector(roundedTens.X()-10, roundedTens.Y());
	        }
	            //std::cout<<"Rounded hole - "<<roundedHole<<std::endl;
	            return roundedHole;
	        }
	}
};

#endif

