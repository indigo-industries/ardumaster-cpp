// Direct handler state regression: protocol type isolation before mutations.
#define main wopr_bridge_server_main
#include "wopr_bridge.cpp"
#undef main
#include <cstdio>
int main() {
 Bridge b; InitPayload init{};init.lat_1e7=398260990;init.lng_1e7=-840483020;init.alt_m=3500;
 if(b.init(init)!=Status::kOk)return 1;
 mavgcs::GcsLink g{};g.sysid=7;
 auto count=[&](){fwcpp::gcs::Frame f{};f.msgid=fwcpp::gcs::kMsgIdMissionCount;fwcpp::gcs::write_u16_le(f.payload.data(),5);f.payload[2]=7;f.payload[3]=1;mavgcs::handle_frame(g,b,f);};
 auto item=[&](int seq,int type,int sys=7,int comp=1){fwcpp::gcs::Frame f{};f.msgid=fwcpp::gcs::kMsgIdMissionItemInt;auto*p=f.payload.data();fwcpp::gcs::write_f32_le(p+4,250);fwcpp::gcs::write_i32_le(p+16,398260990+seq*100000);fwcpp::gcs::write_i32_le(p+20,-840483020);fwcpp::gcs::write_f32_le(p+24,type?4100:4000);fwcpp::gcs::write_u16_le(p+28,seq);fwcpp::gcs::write_u16_le(p+30,16);p[32]=sys;p[33]=comp;p[37]=type;mavgcs::handle_frame(g,b,f);};
 count();for(int i=0;i<5;++i)item(i,0);if(g.cached_count!=5||!b.gcs_set_mode_custom(10))return 2;
 int cases=0;
 for(int type:{1,2,255})for(int expected:{1,4})for(bool out_of_order:{false,true}) {
  count();for(int i=0;i<expected;++i)item(i,0);
  const auto rows=g.rows,cache=g.cached_rows;const auto mode=b.current_mode();const auto active=b.plane().mission.current()->loc;const auto next=b.plane().next_WP_loc;StateReply before{};b.fill_state(before);
  item(out_of_order?7:expected,type);
  StateReply after{};b.fill_state(after);
  if(!g.up_active||g.up_count!=5||g.up_next!=expected||g.cached_count!=5||
    std::memcmp(rows.data(),g.rows.data(),sizeof(rows))||std::memcmp(cache.data(),g.cached_rows.data(),sizeof(cache))||
    before.mission_count!=after.mission_count||b.current_mode()!=mode||b.plane().mission.current()->loc.lat!=active.lat||b.plane().mission.current()->loc.lng!=active.lng||b.plane().mission.current()->loc.alt!=active.alt||b.plane().next_WP_loc.lat!=next.lat||b.plane().next_WP_loc.lng!=next.lng||b.plane().next_WP_loc.alt!=next.alt){std::printf("state changed type=%d expected=%d order=%d next=%u\n",type,expected,out_of_order,g.up_next);return 3;}
  for(int i=expected;i<5;++i)item(i,0);
  if(g.up_active||g.cached_count!=5||g.cached_rows[expected].z!=4000)return 4;
  ++cases;
 }
 std::printf("%d direct handler type-isolation/state/continuation cases PASS\n",cases);
}
