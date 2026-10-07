#include <iostream>
#include <stdexcept>
#include "bbot_balance_controller/landing_repair_handoff.hpp"
using namespace bbot_jump;
void require(bool ok, const char * message) { if (!ok) throw std::runtime_error(message); }
int main() {
    LandingReferenceHandoff h;
    const std::array<double,4> q{.24,-.21,.24,-.21};
    auto previous=h.update(1.,{2.,-10.,2.,-10.},q,{4.,-4.},true,true,true,true);
    require(previous.valid && previous.hip_feedback[0]==0.,"initial feedback discontinuity");
    for(int i=1;i<=30;i++) {
        const bool supported=i<6 || (i>=9 && i<16);
        const auto next=h.update(1.+i*.005,
            supported ? std::array<double,4>{7.,-11.,7.,-11.} : std::array<double,4>{1.9,-3.,1.9,-3.},
            q,{4.,-4.},true,true,true,supported);
        require(next.valid,"bounded contact/release replay rejected");
        for(int k=0;k<4;k++) {
            require(std::abs(next.velocity[k]-previous.velocity[k]) <= (k%2?500.:450.)*.005+1e-10,
                "reference acceleration exceeded");
            require(std::abs(next.hip_feedback[k%2])<=4.,"feedback budget exceeded");
        }
        previous=next;
    }
    require(std::abs(previous.hip_feedback[0])<.1,"release feedback did not decay");
    require(handoff_support_allowed(true,true,3,1.,.999),"fresh bilateral rejected");
    require(!handoff_support_allowed(true,true,0,1.,.999),"zero contact treated as support");
    require(!handoff_support_allowed(true,true,1,1.,.999),"single wheel treated as bilateral");
    require(!handoff_support_allowed(false,true,3,1.,.999),"invalid contact accepted");
    require(!handoff_support_allowed(true,false,3,1.,.999),"contact source gap accepted");
    require(!handoff_support_allowed(true,true,3,1.,.980),"stale contact accepted");
    require(!handoff_support_allowed(true,true,3,1.,1.001),"future contact accepted");
    require(handoff_actual_travel_safe(q,{8.,-13.,8.,-13.}),"safe incoming motion rejected");
    require(!handoff_actual_travel_safe({.24,-1.4,.24,-1.4},{0.,-10.,0.,-10.}),
        "slow reference hid real incoming travel limit");
    require(!h.update(2.,{2.,-10.,2.,-10.},q,{4.,4.},false,true,true,true).valid,
        "stale data allowed carried feedback");
    require(!h.sample().valid,"invalid data left a live seed");
    require(!h.update(2.,{2.,-10.,2.,-10.},q,{4.,4.},true,false,true,true).valid,
        "pending effort switch allowed update");
    require(!h.update(2.,{2.,-10.,2.,-10.},q,{4.,4.},true,true,false,true).valid,
        "travel guard bypassed");
    require(!h.update(2.,{2.,-10.,2.,-10.},{1.50,-1.54,1.50,-1.54},
        {4.,4.},true,true,true,true).valid,"near-stop reference accepted");
    require(h.update(2.,{2.,-10.,2.,-10.},q,{4.,4.},true,true,true,true).valid,"valid restart rejected");
    const auto same=h.update(2.,{7.,-11.,7.,-11.},q,{4.,4.},true,true,true,true);
    require(same.valid && same.velocity[0]==2.,"same-tick FLIGHT advanced reference twice");
    h.reset();
    require(h.update(3.,{2.,-14.,2.,-14.},q,{0.,0.},true,true,true,false).valid,
        "legal THRUST reference rejected");
    require(!h.update(3.,{2.,-12.,2.,-12.},q,{0.,0.},true,true,true,false,13.).valid,
        "same-tick FLIGHT allowed inherited over-speed");
    h.reset();
    h.update(4.,{2.,-14.,2.,-14.},q,{0.,0.},true,true,true,false);
    require(!h.update(4.005,{2.,-12.,2.,-12.},q,{0.,0.},true,true,true,false,13.).valid,
        "filtered FLIGHT allowed inherited over-speed");
    std::cout << "bounded velocity/feedback handoff, contact and failure guards passed\n";
}
