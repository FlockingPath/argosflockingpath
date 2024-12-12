#include "flocking_loop_functions.h"
#include <argos3/core/simulator/simulator.h>
#include <argos3/core/utility/configuration/argos_configuration.h>
#include <argos3/plugins/robots/foot-bot/simulator/footbot_entity.h>
#include <controllers/footbot_flocking/footbot_flocking.h>

/****************************************/
/****************************************/

CFlockingLoopFunctions::CFlockingLoopFunctions() :
   m_cFlockingArenaSideX(-0.9f, 1.7f),
   m_cFlockingArenaSideY(-1.7f, 1.7f),
   m_pcFloor(NULL),
   m_pcRNG(NULL),
   DrawIDs(1),
   m_pcGrid(new Grid(100, 100)) // Initialize grid size
   //m_unCollectedFood(0),
   //m_nEnergy(0),
   //m_unEnergyPerFoodItem(1),
   //m_unEnergyPerWalkingRobot(1) 
   {
}

/****************************************/
/****************************************/

void CFlockingLoopFunctions::Init(TConfigurationNode& t_node) {
   try {
      TConfigurationNode& tFlocking = GetNode(t_node, "flocking");
     // /* Get a pointer to the floor entity */
     // m_pcFloor = &GetSpace().GetFloorEntity();
     // /* Get the number of food items we want to be scattered from XML */
     // UInt32 unFoodItems;
      GetNodeAttribute(tFlocking, "target_distance", target_distance);
     
     // /* Get the number of food items we want to be scattered from XML */
     // GetNodeAttribute(tFlocking, "radius", m_fFoodSquareRadius);
     // m_fFoodSquareRadius *= m_fFoodSquareRadius;
     // /* Create a new RNG */
     // m_pcRNG = CRandom::CreateRNG("argos");
     // /* Distribute uniformly the items in the environment */
     // for(UInt32 i = 0; i < unFoodItems; ++i) {
      //   m_cFoodPos.push_back(
     //       CVector2(m_pcRNG->Uniform(m_cFlockingArenaSideX),
     //                m_pcRNG->Uniform(m_cFlockingArenaSideY)));
     // }
     
      /* Get the output file name from XML */
     // GetNodeAttribute(tFlocking, "output", m_strOutput);
     // /* Open the file, erasing its contents */
      m_cOutput.open(m_strOutput.c_str(), std::ios_base::trunc | std::ios_base::out);
      m_cOutput << "# clock\twalking\tresting\tcollected_food\tenergy" << std::endl;
     // /* Get energy gain per item collected */
     // GetNodeAttribute(tFlocking, "energy_per_item", m_unEnergyPerFoodItem);
     // /* Get energy loss per walking robot */
     // GetNodeAttribute(tFlocking, "energy_per_walking_robot", m_unEnergyPerWalkingRobot); 
   }
   catch(CARGoSException& ex) {
      THROW_ARGOSEXCEPTION_NESTED("Error parsing loop functions!", ex);
   }
   
   argos::CSpace::TMapPerType& footbots = GetSpace().GetEntitiesByType("foot-bot");
	   argos::CSpace::TMapPerType::iterator it;
    
    argos::LOG<<"Number of robots="<<footbots.size()<< ", DrawIDs="<< DrawIDs<<endl;
	   for(it = footbots.begin(); it != footbots.end(); it++) {
   	   	argos::CFootBotEntity& footBot = *argos::any_cast<argos::CFootBotEntity*>(it->second);
		      //BaseController& c = dynamic_cast<BaseController&>(footBot.GetControllableEntity().GetController());
		      //CFootBotFlocking& c2 = dynamic_cast<CFootBotFlocking&>(c);
		      CFootBotFlocking& c2 = dynamic_cast<CFootBotFlocking&>(footBot.GetControllableEntity().GetController());
		      
        c2.SetLoopFunctions(this);
	    }   
	    
	    last_time_in_seconds = 0;
	    
	    m_pcGrid = new Grid(30, 30); // Init grid

        // Set obstacles in the grid
        m_pcGrid->SetObstacle(9, 3);
        m_pcGrid->SetObstacle(0, 0);
        m_pcGrid->SetObstacle(1, 0);
        m_pcGrid->SetObstacle(0, 2);

        // Convert world coordinates to grid coordinates
        SetObstaclesInGrid();
}

/****************************************/
/****************************************/

void CFlockingLoopFunctions::Reset() {
   /* Zero the counters */
   //m_unCollectedFood = 0;
   //m_nEnergy = 0;
   /* Close the file */
   m_cOutput.close();
   /* Open the file, erasing its contents */
   m_cOutput.open(m_strOutput.c_str(), std::ios_base::trunc | std::ios_base::out);
   m_cOutput << "# clock\twalking\tresting\tcollected_food\tenergy" << std::endl;
   /* Distribute uniformly the items in the environment */
   //for(UInt32 i = 0; i < m_cFoodPos.size(); ++i) {
   //   m_cFoodPos[i].Set(m_pcRNG->Uniform(m_cFlockingArenaSideX),
   //                     m_pcRNG->Uniform(m_cFlockingArenaSideY));
  // }
}

/****************************************/
/****************************************/

void CFlockingLoopFunctions::Destroy() {
   /* Close the file */
   m_cOutput.close();
   delete m_pcGrid; // Free allocated mem for Grid
}

/****************************************/
/****************************************/

CColor CFlockingLoopFunctions::GetFloorColor(const CVector2& c_position_on_plane) {
   if(c_position_on_plane.GetX() < -1.0f) {
      return CColor::GRAY50;
   }
   for(UInt32 i = 0; i < m_cFoodPos.size(); ++i) {
      if((c_position_on_plane - m_cFoodPos[i]).SquareLength() < m_fFoodSquareRadius) {
         return CColor::BLACK;
      }
   }
   return CColor::WHITE;
}


Real CFlockingLoopFunctions::getSimTimeInSeconds() {
	int ticks_per_second = GetSimulator().GetPhysicsEngine("dyn2d").GetInverseSimulationClockTick(); //qilu 02/06/2021
	float sim_time = GetSpace().GetSimulationClock();
	return sim_time/ticks_per_second;
}


map<string, CVector2> CFlockingLoopFunctions::GetRobotPosition()
{
	return robotPos;
	}


void CFlockingLoopFunctions::CheckConnectivity() {
   //  for (auto& robot : m_tRobotStates) {
   //      CFootBotFlocking& controller = dynamic_cast<CFootBotFlocking&>(robot.second->GetController());
   //      const CCI_PositioningSensor::SReading& position = controller.GetPositioningSensor().GetReading();
   //      CVector2 robotPos(position.Position.GetX(), position.Position.GetY());

   //      bool isConnected = false;
   //      for (auto& otherRobot : m_tRobotStates) {
   //          if (robot.first != otherRobot.first) {
   //              CFootBotFlocking& otherController = dynamic_cast<CFootBotFlocking&>(otherRobot.second->GetController());
   //              const CCI_PositioningSensor::SReading& otherPosition = otherController.GetPositioningSensor().GetReading();
   //              CVector2 otherRobotPos(otherPosition.Position.GetX(), otherPosition.Position.GetY());

   //              if ((robotPos - otherRobotPos).Length() < m_fCommunicationRange) {
   //                  isConnected = true;
   //                  break;
   //              }
   //          }
   //      }
   //  }
}

/****************************************/
/****************************************/
void CFlockingLoopFunctions::IncreaseTargetDistance()
{
	target_distance++;
	}

UInt32 CFlockingLoopFunctions::GetTargetDistance()
{
	return target_distance;
	}

/****************************************/
/****************************************/

void CFlockingLoopFunctions::PreStep() {
   /* Logic to pick and drop food items */
   /*
    * If a robot is in the nest, drop the food item
    * If a robot is on a food item, pick it
    * Each robot can carry only one food item per time
    */
   //UInt32 unWalkingFBs = 0;
   //UInt32 unRestingFBs = 0;
   /* Check whether a robot is on a food item */
   CSpace::TMapPerType& m_cFootbots = GetSpace().GetEntitiesByType("foot-bot");
   
   robotPos.clear();
   bool collide_flag = 0;
   for(CSpace::TMapPerType::iterator it = m_cFootbots.begin();
       it != m_cFootbots.end();
       ++it) {
      /* Get handle to foot-bot entity and controller */
      CFootBotEntity& cFootBot = *any_cast<CFootBotEntity*>(it->second);
      CFootBotFlocking& cController = dynamic_cast<CFootBotFlocking&>(cFootBot.GetControllableEntity().GetController());
     
      /* Get the position of the foot-bot on the ground as a CVector2 */
      if(cController.GetStatus() == 1){
		  collide_flag = 1;
		  }
      
      CVector2 cPos;
      cPos.Set(cFootBot.GetEmbodiedEntity().GetOriginAnchor().Position.GetX(),
               cFootBot.GetEmbodiedEntity().GetOriginAnchor().Position.GetY());
      robotPos[cController.GetId()] = cPos;  
      
      LOG << "ID="<< cController.GetId() << ", pos = " << cPos << endl;
     
     //for(map<string, double>::iterator it= cController.connectedRobots.begin(); it!= cController.connectedRobots.end(); ++it) {
	 //  LOG<< "connected "<< it->first << endl;
	  // }  
   }
   
   /* Get handle to the first foot-bot entity and controller */
   CFootBotEntity& cFootBot = *any_cast<CFootBotEntity*>(m_cFootbots.begin()->second);
   CFootBotFlocking& cController = dynamic_cast<CFootBotFlocking&>(cFootBot.GetControllableEntity().GetController());
   
    for(map<string, double>::iterator it= cController.connectedRobots.begin(); it!= cController.connectedRobots.end(); ++it) {
	  LOG<< "selected ID="<< cController.GetId() << ", connected "<< it->first << endl;
	 }   
   
   
   //increase the distance to neighbors
   curr_time_in_seconds = getSimTimeInSeconds();
      if(collide_flag == 1 && curr_time_in_seconds - last_time_in_seconds >= 2 && curr_time_in_seconds > 0){
      //LOG<<"current time = "<< curr_time_in_seconds<< endl;
      if(target_distance < 200){
			IncreaseTargetDistance();
      }
      //LOG<<"TargetDistance = "<< target_distance<<endl;
      last_time_in_seconds = curr_time_in_seconds;
      }

    // check the connectivity of the swarm
    CheckConnectivity();
	
  // /* Output stuff to file */
   m_cOutput << GetSpace().GetSimulationClock() << "\t"
    //         << unWalkingFBs << "\t"
    //         << unRestingFBs << "\t"
    //         << m_unCollectedFood << "\t"
    //         << m_nEnergy 
    << std::endl;
    
    // Example usage of Dijkstra algorithm
    Node start = {0, 0, 0, nullptr};
    Node target = {10, 10, 0, nullptr};
    std::vector<Node> path = m_pcGrid->CalcShortestPath(start.x, start.y, target.x, target.y);
    // Use the path for navigation

   //dynamicobs
    static int step_counter = 0;
    if (step_counter % 100 == 0) { // Update every 100 steps
        UpdateObstaclePositions();
    }
    step_counter++;
}

/****************************************/
/****************************************/

// Function to convert world coordinates to grid coordinates
std::pair<int, int> CFlockingLoopFunctions::WorldToGridCoordinates(const CVector3& c_position) {
    int gridX = static_cast<int>(c_position.GetX() + 15); // Assuming grid center is at (15, 15)
    int gridY = static_cast<int>(c_position.GetY() + 15);
    return {gridX, gridY};
}

// Function to set obstacles in the grid
void CFlockingLoopFunctions::SetObstaclesInGrid() {
    // Example obstacles from the XML configuration
    std::vector<CVector3> obstaclePositions = {
        CVector3(9, 3, 0),
        CVector3(0, 0, 0),
        CVector3(1, 0, 0),
        CVector3(0, 2, 0)
    };

    for (const auto& position : obstaclePositions) {
        auto [gridX, gridY] = WorldToGridCoordinates(position);
        m_pcGrid->SetObstacle(gridX, gridY);
    }
}

/****************************************/
/****************************************/
//dynamicobs
void CFlockingLoopFunctions::UpdateObstaclePositions() {
    // Clear the current obstacles in the grid
    m_pcGrid->ResetGrid();

    // Loop through all obstacles and change their positions randomly
    for (auto& obstacle : m_cFoodPos) {
        // Randomly update obstacle position (within bounds)
        Real random_x = m_pcRNG->Uniform(CRange<Real>(m_cFlockingArenaSideX.GetMin(), m_cFlockingArenaSideX.GetMax()));
        Real random_y = m_pcRNG->Uniform(CRange<Real>(m_cFlockingArenaSideY.GetMin(), m_cFlockingArenaSideY.GetMax()));

        // Set the new position for the obstacle
        obstacle.SetX(random_x);
        obstacle.SetY(random_y);

        // Update the grid to reflect the new obstacle position
        auto grid_coords = WorldToGridCoordinates(CVector3(random_x, random_y, 0));
        m_pcGrid->SetObstacle(grid_coords.first, grid_coords.second);
    }
}

/****************************************/
/****************************************/

REGISTER_LOOP_FUNCTIONS(CFlockingLoopFunctions, "flocking_loop_functions")
