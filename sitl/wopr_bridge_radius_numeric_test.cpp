#define main wopr_bridge_server_main
#include "wopr_bridge.cpp"
#undef main
#include <limits>
int main(){
 Bridge b;InitPayload init{};init.lat_1e7=398260990;init.lng_1e7=-840483020;init.alt_m=3500;if(b.init(init)!=Status::kOk)return 1;
 std::array<Bridge::GcsGeoItem,4> rows{};for(int i=0;i<4;++i){auto&r=rows[i];r.seq=i;r.command=16;r.frame=0;r.p2=250;r.p4=std::numeric_limits<float>::quiet_NaN();r.lat_1e7=398260990+i*100000;r.lng_1e7=-840483020;r.z=4000;}
 if(b.gcs_mission_load(rows.data(),rows.size())!=3||!b.gcs_set_mode_custom(10))return 2;
 if(!const_cast<fwcpp::vehicle::Plane&>(b.plane()).mission.advance())return 9;
 int cases=0;
 for(int frame:{0,3,5,6})for(int seq:{1,3})for(float value:{std::numeric_limits<float>::quiet_NaN(),std::numeric_limits<float>::infinity(),-std::numeric_limits<float>::infinity()}){
  auto bad=rows;bad[seq].frame=frame;bad[seq].p2=value;const auto original=b.plane().mission.current()->loc;const auto mode=b.current_mode();StateReply before{};b.fill_state(before);int result=b.gcs_mission_load(bad.data(),bad.size());
  if(result!=-7){std::printf("bad p2 accepted frame=%d seq=%d result=%d\n",frame,seq,result);return 3;}
  StateReply state{};b.fill_state(state);const auto active=b.plane().mission.current()->loc;
  if(state.mission_index!=before.mission_index||state.mission_count!=3||b.current_mode()!=mode||active.lat!=original.lat||active.lng!=original.lng||active.alt!=original.alt||b.plane().mission.current()->acceptance_radius_m!=250)return 4;
  ++cases;
 }
 for(float value:{0.0f,-1.0f,250.0f,std::numeric_limits<float>::max()}){
  auto valid=rows;valid[1].p2=value;if(b.gcs_mission_load(valid.data(),valid.size())!=3)return 5;
  if(b.plane().mission.current()->acceptance_radius_m!=std::fmax(0.0f,value))return 6;
 }
 for(float value:{std::numeric_limits<float>::quiet_NaN(),std::numeric_limits<float>::infinity()}){auto home=rows;home[0].p2=value;if(b.gcs_mission_load(home.data(),home.size())!=3)return 7;}
 auto unused=rows;unused[1].command=22;unused[1].p2=std::numeric_limits<float>::infinity();if(b.gcs_mission_load(unused.data(),unused.size())!=3)return 8;
 std::printf("%d nonfinite consumed-radius rejections and7 valid/ignored compatibility cases PASS\n",cases);
}
