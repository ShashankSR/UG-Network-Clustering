/*
  This code is develop as part of course project in NITK - IT Department
  Authors :- 
    Anuj kumar, 
    Shashank S R, 
    Shashi Roshan 
  NOTE: This program uses modified lr-wpan model of ns3. Pls see readme for instructions.
*/


//TODO: energy vairations based on distance, SNR, sleep and wake up cycles
//NOTE TO SELF : INCREASE IN SIZE MAKES MULTIPLE PACKETS ->COLLISION->PACKLOSS
//COLLISIONS ARE MAIN REASON FOR NO PACKET
//INCREASED TIME SLOTS TO AVOID COLLISIONS
 
#include <ns3/log.h>
#include <ns3/core-module.h>
#include <ns3/lr-wpan-module.h>
#include <ns3/propagation-loss-model.h>
#include <ns3/propagation-delay-model.h>
#include <ns3/simulator.h>
#include <ns3/realtime-simulator-impl.h>
#include <ns3/single-model-spectrum-channel.h>
#include <ns3/constant-position-mobility-model.h>
#include <ns3/packet.h>
#include <ns3/simple-device-energy-model.h>
#include <ns3/li-ion-energy-source.h>
#include <ns3/energy-source-container.h>


#include <iostream>
#include <string>
#include <sstream>
#include <algorithm>
#include <iterator>
#include <string>
#include <fstream>
#include <cmath>
#include <inttypes.h>

using namespace ns3;

// ----------------------------------------------------------------------------------------//

#define ADV_CH           0
#define JOIN_REQ         1
#define ADV_SCH          2
#define DATA             3

#define SEND             10
#define RECIEVE          12

#define NUM_OF_CLUSTERS  5
#define NUM_NODES        100

#define EQ_ENERGY        1
#define TRACE_ALGO       false
#define SHOW_RECIEVED_LOG false

#define MAX_DISTANCE    300

#define ENERGY 		       2
//#define SEND_ENERGY	     50e-6
//#define RECV_NRG_RATE	   50e-6
#define SEND_ENERGY      50e-9
#define RECV_NRG_RATE    50e-9
#define SLEEP_NRG	       5e-12
#define IDLE_NRG	       100e-12
#define AMP_ENR_RATE 	   100e-12

//------------------------Need to remove global variables------------------------------------------------//

int cluster_count_ = 0;
std::string mDist[100][2];
bool roundDone = true;
int global_round_ = 0;

static void StateChangeNotification (std::string context, Time now, LrWpanPhyEnumeration oldState, LrWpanPhyEnumeration newState)
{
// Too much logging information.
/*NS_LOG_UNCOND (context << " state change at " << now.GetSeconds () 
                         << " from " << LrWpanHelper::LrWpanPhyEnumerationPrinter (oldState)
                         << " to " << LrWpanHelper::LrWpanPhyEnumerationPrinter (newState)); */
}



// All member variables public for simplicity. Change accordingly if you want as private.

class leach {

public:
    
    float locX;
    float locY;
    float thresh;
    float energy;
    int clustDist;
    int nodeId;
    int clusterHeadId;
    int round_ ;
    int ch_rotate;
    int packetsReceived;
    bool IsClusterHead;

    Mac16Address ch_addr;
    std::vector<int> buffer;    
    uint8_t *cluster_table;
    Ptr<Node> n0;
    Ptr<LrWpanNetDevice> dev0;
    
    Ptr<LiIonEnergySource> es;
    Ptr<SimpleDeviceEnergyModel> sem;

    McpsDataRequestParams m_Params;
    Ptr<Packet> m_Packet ;

    leach(){ }


//-----------------Constructor object for Mote -------------------------------------------//

    leach(float locX,float locY,Mac16Address mac_addr,int id,float energy) {
      
      n0 = CreateObject <Node> ();
      dev0 = CreateObject<LrWpanNetDevice> ();

      sem = CreateObject<SimpleDeviceEnergyModel> ();
      Ptr<EnergySourceContainer> esCont = CreateObject<EnergySourceContainer> ();
      es = CreateObject<LiIonEnergySource> ();
      
      esCont->Add (es);
      es->SetNode (n0);
      sem->SetEnergySource (es);
      es->AppendDeviceEnergyModel (sem);
      sem->SetNode (n0);
      n0->AggregateObject (esCont);
      sem->SetCurrentA (12e-4 + (float)rand()/RAND_MAX/10000);
      // Note since we are using extended time slots, current is e-4 instead of e-3
      
      dev0->SetAddress (mac_addr);

      n0->AddDevice (dev0);
      dev0->GetPhy ()->TraceConnect ("TrxState", std::string ("phy0"), MakeCallback (&StateChangeNotification));

      Ptr<ConstantPositionMobilityModel> sender0Mobility = CreateObject<ConstantPositionMobilityModel> ();
      sender0Mobility->SetPosition (Vector (locX,locY,0));
      dev0->GetPhy ()->SetMobility (sender0Mobility);
      
      round_ = 0;
      nodeId = id;
      ch_rotate = 0;
      IsClusterHead = false;

      this->locY = locY;
      this->locX = locX;
      this->energy = energy;
      
      if (TRACE_ALGO)
      {
        std::cout<<"Node created with Mac Addrs :" << mac_addr <<" Battery-level :"<< 
             es->GetSupplyVoltage( ) <<"v  positioned at " << locX <<"  " <<locY<< "Node Id :: " << nodeId<<"\n";
      }
     
    }


    //----------------------------Helper function----------------------------------------------------//

    Ptr<LrWpanNetDevice> getDevice(){
      return dev0;
    }

    //-------------TODO----------------//
    void setSleepCurrent(){
        sem->SetCurrentA (3e-4 + (float)rand()/RAND_MAX/100000);
    }

    void setIdleCurrent(){
        sem->SetCurrentA (12e-4 + (float)rand()/RAND_MAX/100000);
    }

    void reduceEnergy(int action){
      if(action == SEND){
        // TODO
      }else if(action == RECIEVE){
        // TODO
      }
    }

    bool checkAlive(){

      DoubleValue v;
      es->GetAttribute ("ThresholdVoltage", v);
      if (es->GetSupplyVoltage () <= v.Get ()){
        return true;
      }

      return false;
    }
    //----------------------------------------------------------------------------------------------//

    void FormClusters ( ) {

    //Calculate threshold energy
    //Generate a random number & if > Thres and numcluster < max assign clusterhead advertise
    //else recieve advertise and find Best cluster
      
      if (EQ_ENERGY == 1) {
        thresh = (float)NUM_OF_CLUSTERS /  ((float)NUM_NODES );//- (float)NUM_OF_CLUSTERS*round_);
        if(IsClusterHead){ thresh = 0; ch_rotate = NUM_OF_CLUSTERS; IsClusterHead = false;}
        if(ch_rotate != 0){ thresh = 0; ch_rotate --;}
      } else {
        //TODO
        // This case occurs when nodes are deployed with unequal energy. A resource manager is 
        // required to maintain the energy values of the network. 
      }

      float r = ((float) rand() / (RAND_MAX))/10;
      
      if ( ( r < thresh) && (cluster_count_ < (global_round_ + 1)*NUM_OF_CLUSTERS) ) {
       
        // Carefully set the time delays .. Understand the concept of 'DESCRETE Event' based
        // simulation to understand why the code is this way.
        // There will be a lot of collisions as time passes so be careful 

        IsClusterHead = true;

        Simulator::Schedule ( Seconds (Simulator::Now().GetSeconds() + 0.1 + global_round_*10),
                              &leach::SetClusterhead, this);

       
        Simulator::Schedule ( Seconds (Simulator::Now().GetSeconds() +0.2+(float)rand()/RAND_MAX+ global_round_*10),
                              &LrWpanMac::McpsDataRequestHelper, dev0->GetMac (), 
                              &m_Params, &m_Packet);

        Simulator::Schedule ( Seconds (Simulator::Now().GetSeconds() +0.5+(float)rand()/RAND_MAX+ global_round_*10 ),
                              &leach::ReduceSendEnergy, this);
        
        // broadcast the TDMA schedule 
        Simulator::Schedule ( Seconds (Simulator::Now().GetSeconds() +3+(float)rand()/RAND_MAX + global_round_*10 ),
                              &LrWpanMac::McpsDataRequestHelper, dev0->GetMac (), 
                              &m_Params, &m_Packet);

        Simulator::Schedule ( Seconds (Simulator::Now().GetSeconds() +3+(float)rand()/RAND_MAX + global_round_*10 ),
                              &leach::ReduceSendEnergy, this);
        

        // Send the data packet to base station
        Simulator::Schedule ( Seconds (Simulator::Now().GetSeconds() + 10 + global_round_*10 ),
                        &leach::sendToBs, this);

     
        Simulator::Schedule ( Seconds (Simulator::Now().GetSeconds() + 10 + global_round_*10),
                              &leach::ReduceSendEnergy, this);
        


        cluster_count_ ++;
       
        
      } else {

 
        Simulator::Schedule ( Seconds (Simulator::Now().GetSeconds()+ global_round_*10 ),
                              &leach::UnsetClusterhead, this);
        
        Simulator::Schedule ( Seconds (Simulator::Now().GetSeconds() + (float)rand()/RAND_MAX +0.05  + 1 + global_round_*10),
                              &LrWpanMac::McpsDataRequestHelper, dev0->GetMac (), 
                              &m_Params, &m_Packet);


        Simulator::Schedule ( Seconds (Simulator::Now().GetSeconds() + (float)rand()/RAND_MAX +0.05  +0.5 + global_round_*10),
                              &leach::ReduceSendEnergy, this);

       // Send the data to the cluster head and it will also be updated in the callback
        Simulator::Schedule ( Seconds (Simulator::Now().GetSeconds() + 4 + (float)rand()/RAND_MAX*5 + global_round_*10),
                              &LrWpanMac::McpsDataRequestHelper, dev0->GetMac (), 
                              &m_Params, &m_Packet);

        Simulator::Schedule ( Seconds (Simulator::Now().GetSeconds() + 4 + (float)rand()/RAND_MAX*5 + global_round_*10),
                              &leach::ReduceSendEnergy, this);
                              
                              
      }

      round_ ++;
    }
       
    //-------------------------------------------------------------------------------------------//

    void DataReceived (McpsDataIndicationParams params, Ptr<Packet> p){

      LrWpanMacHeader macHdr;
      p->RemoveHeader(macHdr);

      int srcId = macHdr.GetSrcPanId();     //Get src id and addr
      int type = macHdr.GetData();

      Mac16Address srcAddr = macHdr.GetShortSrcAddr();


      switch(macHdr.GetData()){
        case ADV_CH : 
                   
                    //if not clusterhead then calculate distance
                    if(!IsClusterHead){
                        
                        if(SHOW_RECIEVED_LOG){
                         std::cout<<nodeId<<" received  packet of type "<<type<<" from source "<< srcAddr
                         <<" at time "<<Simulator::Now().GetSeconds()<<"\n";
                        }

                        // Selecting clusterhead based on distance for now
                        // Assuming signal strenght is greatest for nearest ch.

                       int dist = std::sqrt(std::pow((std::atof(mDist[nodeId][0].c_str())-
                                    std::atof(mDist[srcId][0].c_str())),2) + 
                                std::pow((std::atof(mDist[nodeId][1].c_str())-
                                  std::atof(mDist[srcId][1].c_str())),2));
                        
                        if(clustDist > dist ) {
                              clustDist = dist;
                              ch_addr = srcAddr;      
                        }
                        
                        setPacket(JOIN_REQ,ch_addr,dev0->GetMac()->GetShortAddress(),2,NULL,10);                     
                    }
                    break;
                    
        case JOIN_REQ:
                    //if clusterhead then join into clusterlist

                    if(IsClusterHead){

                      if(SHOW_RECIEVED_LOG){
                         std::cout<<nodeId<<" received  packet of type "<<type<<" from source "<< srcAddr
                           <<" at time "<<Simulator::Now().GetSeconds()<<"\n";
                      }
                      

                      buffer.push_back(srcId); 
                      setTDMA();
                    }

                    break;
        case ADV_SCH :
                    //if not clusterhead and under this clusterhead
                    // store TDMA values and send data
                    if(srcAddr == ch_addr){

                        if(SHOW_RECIEVED_LOG){
                          std::cout<<nodeId<<" received  packet of type "<<type<<" from source "<< srcAddr
                          <<" at time "<<Simulator::Now().GetSeconds()<<"\n";
                        }


                        uint8_t *b = (uint8_t*) malloc(8*p->GetSize()); 
                    	  p->CopyData(b,p->GetSize()); // get the TDMA.
                    	
                     	  int i;
                    	  for(i=0;(int)b[i] != nodeId;i++);
                    	  energy -= (SLEEP_NRG*i*0.4);  

                        setPacket(DATA,ch_addr,dev0->GetMac()->GetShortAddress(),64,NULL,MAX_DISTANCE);    
                    	// sleep till i*slot from now and send at 1*slot

                    }
                    break;
        case DATA:
                  if (IsClusterHead ){
                      if(SHOW_RECIEVED_LOG){
                          std::cout<<nodeId<<" (CH node) received  packet of type "<<type<<" from source "<< srcAddr
                           <<" at time "<<Simulator::Now().GetSeconds()<<"\n";   
                      }
                      packetsReceived ++;

                  }
                  break;


      }
     
      energy -= RECV_NRG_RATE*p->GetSize();
    }

    //---------------------------------------------------------------------------------------//


    void setPacket(int type,Mac16Address dst_addr,Mac16Address src_addr,
                    int size,uint8_t buffer[],float dist){
   //   std::cout<<"Setting packet from node "<< nodeId <<"\n";

      if(buffer){      
        m_Packet = Create<Packet> (buffer,size);
      }else{
        m_Packet = Create<Packet> (size);
      }

      LrWpanMacHeader macHdr (LrWpanMacHeader::LRWPAN_MAC_BEACON, 0);        //sequence number set to 0
      macHdr.SetSrcAddrMode (2);                                             // short addr
      macHdr.SetDstAddrMode (2);
      macHdr.SetSrcAddrFields(nodeId, src_addr );
      macHdr.SetDstAddrFields(0,dst_addr );
      macHdr.SetData(type);
      m_Packet->AddHeader(macHdr);

    
      m_Params.m_srcAddrMode = SHORT_ADDR;
      m_Params.m_dstAddrMode = SHORT_ADDR;
      m_Params.m_dstAddr = Mac16Address (dst_addr);
      m_Params.m_msduHandle = 0;
      m_Params.m_txOptions = TX_OPTION_ACK;

 //     energy -= (float)(SEND_ENERGY*size + AMP_ENR_RATE*dist*dist*size);  

    }


    void setTDMA(){

    	cluster_table = (uint8_t*)malloc(buffer.size()*sizeof(uint8_t));
      	for (std::vector<int>::size_type  i = 0; i < buffer.size(); ++i)
      	{
      		cluster_table[i] = (uint8_t)buffer[i];
      		
      	}
      	Mac16Address dst_addr0 ("ff:ff");
      	Mac16Address src_addr0 = dev0->GetMac()->GetShortAddress();
      	setPacket(ADV_SCH,dst_addr0,src_addr0,buffer.size(),cluster_table,MAX_DISTANCE); // send TDMA pack
    }

    void sendToBs(){
    //	std::cout<<" \n CH node "<<nodeId<<" sending data to base. Total packets received  "<<packetsReceived<<" From "<<buffer.size()<<" nodes " ;
      energy -= SEND_ENERGY*50;
    //	std::cout<<" Remaining energy :: "<<energy<< " at time "<<Simulator::Now().GetSeconds() ;
      buffer.clear();
    }

  void SetClusterhead(){
 
    IsClusterHead = true;
    packetsReceived = 0;
    Mac16Address dst_addr ("ff:ff");
    setPacket(ADV_CH,dst_addr,dev0->GetMac()->GetShortAddress(),2,NULL,MAX_DISTANCE);
     clustDist = MAX_DISTANCE;
  }

  void ReduceSendEnergy(){
    energy -= SEND_ENERGY*4;
  }
    void UnsetClusterhead(){
    IsClusterHead = false;
    clustDist = MAX_DISTANCE;
  }

} ;
//---------------------------------------------------------------------------------------------

void NetworkEnergy(leach leach_nodes[],int round_){
  float total_energy = 0; 
  for (int i = 0; i < NUM_NODES; ++i) {
    total_energy += leach_nodes[i].energy; 

  }
  std::cout<<"\nAverage network energy after round "<< round_ << " is : "<<total_energy/NUM_NODES<<"\n\n";
  //std::cout<<"  "<<leach_nodes[10].es->GetRemainingEnergy() <<"\n";
}


//------------------------------------------------------------------------------------------------
int main(int argc, char const *argv[])
{
  //create 100 nodes with leach.

  Ptr<SingleModelSpectrumChannel> channel = CreateObject<SingleModelSpectrumChannel> ();
  Ptr<LogDistancePropagationLossModel> propModel = CreateObject<LogDistancePropagationLossModel> ();
  Ptr<ConstantSpeedPropagationDelayModel> delayModel = CreateObject<ConstantSpeedPropagationDelayModel> ();
  channel->AddPropagationLossModel (propModel);
  channel->SetPropagationDelayModel (delayModel);

  leach leach_nodes[100];
  
  std::ifstream file("100nodes.txt");
  std::string str; 
  //int i=0;
   if (file.is_open())
    {
        for(int i = 0; i < 100; ++i)
        {
            file >> mDist[i][0];
            file >> mDist[i][1];
          
        }
   }
   

  McpsDataIndicationCallback cb[100];
  
  for (int i = 0; i < 100; ++i)
  {
    // TODO:  Dirty hack for setting mac address .. Needs changing

    Packet::EnablePrinting();
    std::stringstream out;
    out << i;
    std::string s = out.str();
    std::string addr;

    if(i<10){ s.insert(0,"00:0");}
    else    { s.insert(0,"00:");}

    //-------------------------------------------------------

    Mac16Address mac_addr (s.c_str());
    leach n1 (std::atof(mDist[i][0].c_str()),std::atof(mDist[i][1].c_str()),mac_addr,i,ENERGY);
    n1.getDevice()->SetChannel(channel);
    

    leach_nodes[i] = n1;

  
    cb[i] = MakeCallback (&leach::DataReceived,&leach_nodes[i]);
    leach_nodes[i].getDevice()->GetMac()->SetMcpsDataIndicationCallback (cb[i]);  
    
  }
 GlobalValue::Bind ("SimulatorImplementationType",
   StringValue ("ns3::RealtimeSimulatorImpl"));

  for (int j = 0; j < 5; ++j) {
      for (int i = 0; i < 100; ++i) {
        leach_nodes[i].FormClusters();
      }
      Simulator::Schedule(Seconds(Simulator::Now().GetSeconds() + global_round_*10 ),&NetworkEnergy,leach_nodes,j);
      global_round_ ++;
  }
 


  Simulator::Run();

  Simulator::Destroy ();
 
  return 0;
}