#include <cstdio>
#include <cmath>
#include <algorithm>
#include "foc_q15.h"
using namespace focq15;
static const float PI_F=3.14159265358979f, SQRT3_F=1.73205080756888f;
struct ABCf{float a,b,c;};
static ABCf abToAbc(float va,float vb){return {va,-0.5f*va+0.5f*SQRT3_F*vb,-0.5f*va-0.5f*SQRT3_F*vb};}
int main(){
  double worst=0; int wth=0; float wm=0;
  for(int k=0;k<65536;k+=13){
    uint16_t th=(uint16_t)k;
    for(float m:{0.1f,0.5f,0.9f,1.0f}){
      float va=m*cosf(2*PI_F*k/65536.0f), vb=m*sinf(2*PI_F*k/65536.0f);
      ABCf p=abToAbc(va*0.7f,vb*0.7f);
      float vmax=std::max(p.a,std::max(p.b,p.c)), vmin=std::min(p.a,std::min(p.b,p.c));
      float off=-0.5f*(vmax+vmin);
      float dF[3]={0.5f+(p.a+off),0.5f+(p.b+off),0.5f+(p.c+off)};
      uint16_t dQ[3];
      svpwm(fromFloat(va),fromFloat(vb),fromFloat(1.0f/0.7f),&dQ[0],&dQ[1],&dQ[2]);
      for(int i=0;i<3;i++){
        float a=toFloat((q15_t)(dQ[i]/2));
        double e=fabs(dF[i]-a);
        if(e>worst){worst=e;wth=k;wm=m;
          printf("worst th=%d(%.1fdeg) m=%.2f i=%d  float=%.4f  q15=%.4f  raw=%u\n",
                 k,360.0*k/65536.0,m,i,dF[i],a,(unsigned)dQ[i]);}
      }
    }
  }
  printf("max duty err=%.5f\n",worst);
  return 0;
}
