#include <gz/sim/System.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Joint.hh>
#include <gz/sim/Util.hh>
#include <gz/plugin/Register.hh>
#include <gz/math/Vector3.hh>
#include <fstream>
#include <iostream>
class PipeControl : public gz::sim::System, public gz::sim::ISystemConfigure, public gz::sim::ISystemPreUpdate {
 gz::sim::Model model{gz::sim::kNullEntity}; gz::sim::Link body{gz::sim::kNullEntity}; bool fired=false; bool continuous=true; double start=0, thrust=3, duration=2, triggerX=.40, nextLog=0; std::ofstream csv;
 public:
 void Configure(const gz::sim::Entity &e,const std::shared_ptr<const sdf::Element> &sdf,gz::sim::EntityComponentManager &ecm,gz::sim::EventManager &) override {
  model=gz::sim::Model(e); std::string name="front"; if(sdf->HasElement("body_link")) name=sdf->Get<std::string>("body_link"); body=gz::sim::Link(model.LinkByName(ecm,name));
  if(sdf->HasElement("thrust")) thrust=sdf->Get<double>("thrust"); if(sdf->HasElement("duration")) duration=sdf->Get<double>("duration"); if(sdf->HasElement("continuous")) continuous=sdf->Get<bool>("continuous"); if(sdf->HasElement("trigger_x")) triggerX=sdf->Get<double>("trigger_x");
  if(sdf->HasElement("telemetry")){csv.open(sdf->Get<std::string>("telemetry")); csv<<"time,x,y,z,forward_z,gas_on,model_x,model_y,model_z\n";}
 }
 void PreUpdate(const gz::sim::UpdateInfo &info,gz::sim::EntityComponentManager &ecm) override {
  if(info.paused) return;
  for(int i=0;i<6;i++){auto ent=model.JointByName(ecm,"wheel_joint_"+std::to_string(i)); if(ent!=gz::sim::kNullEntity) gz::sim::Joint(ent).SetForce(ecm,{0.004});}
  if(!body.Valid(ecm)) return;
  auto p=gz::sim::worldPose(body.Entity(),ecm); double now=std::chrono::duration<double>(info.simTime).count(); auto forward=p.Rot().RotateVector(gz::math::Vector3d(1,0,0));
  // Compare at the same virtual front station, including the single rigid chassis.
  auto station=p.Pos(); if(body.Name(ecm)=="chassis") station+=p.Rot().RotateVector(gz::math::Vector3d(.0325,0,0));
  if(!fired && station.X()>=triggerX){fired=true; start=now; std::cerr<<"CO2 ON t="<<now<<" x="<<station.X()<<" force="<<thrust<<"N\n";}
  bool active=fired && (continuous || now-start<duration);
  if(active) body.AddWorldForce(ecm,forward*thrust);
  if(csv && now>=nextLog){auto mp=gz::sim::worldPose(model.Entity(),ecm); csv<<now<<','<<station.X()<<','<<station.Y()<<','<<station.Z()<<','<<forward.Z()<<','<<active<<','<<mp.Pos().X()<<','<<mp.Pos().Y()<<','<<mp.Pos().Z()<<'\n'; csv.flush(); nextLog=now+.05;}
 }
};
GZ_ADD_PLUGIN(PipeControl,gz::sim::System,PipeControl::ISystemConfigure,PipeControl::ISystemPreUpdate)
GZ_ADD_PLUGIN_ALIAS(PipeControl,"PipeControl")
