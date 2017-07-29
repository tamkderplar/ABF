#include "restrictedregion.h"
#include "misc/misc.h"

RestrictedRegion::RestrictedRegion(Contact c, QLineF obj, QLineF obs)
{
    Contact ev1{obj,obs.p1(),Contact::EdgeVertex}, ev2{obj,obs.p2(),Contact::EdgeVertex};
    Contact ve1{obs,obj.p1(),Contact::VertexEdge}, ve2{obs,obj.p2(),Contact::VertexEdge};
    Q_ASSERT(!(ev1==c || ev2==c || ve1==c || ve2==c));
    DoubleContactFunction dcfs[4] = {{c,ev1},{c,ev2},{c,ve1},{c,ve2}};
    DoubleContactFunction invs[4] = {{ev1,c},{ev2,c},{ve1,c},{ve2,c}};

    QLineF statedge = c.type==Contact::EdgeVertex?obj:obs;
    float s[2] = {  cross2({c.edge.p1(),statedge.p1()},c.edge)/c.edge.length(),
                    cross2({c.edge.p1(),statedge.p2()},c.edge)/c.edge.length()};
    int isVE = c.type==Contact::VertexEdge;
    float undef = *dcfs[2-2*isVE].undefinedValue();
    bool uflip = false, tflip = false;
    if(undef>=glm::pi<float>()){
        undef-=glm::pi<float>();
        uflip = true;
    }
    float theta = std::atan2(cross2(obj,obs), dot2(obj,obs));
    theta = flipToRange(0,glm::two_pi<float>(),theta);
    if(theta>=glm::pi<float>()){
        theta-=glm::pi<float>();
        tflip = true;
    }
    //linear
    int order = ((s[0]<s[1]) != tflip);
    DoubleContactFunction lin1 = dcfs[order+2*isVE];
    DoubleContactFunction lin2 = dcfs[1-order+2*isVE];
    DoubleContactFunction invlin1 = invs[order+2*isVE];
    DoubleContactFunction invlin2 = invs[1-order+2*isVE];
    //rational
    int sign = (uflip!=tflip);
    DoubleContactFunction rat1 = dcfs[sign+2-2*isVE];
    DoubleContactFunction rat2 = dcfs[1-sign+2-2*isVE];
    DoubleContactFunction invrat1 = invs[sign+2-2*isVE];
    DoubleContactFunction invrat2 = invs[1-sign+2-2*isVE];

    float theta0 = qMin(undef, theta);
    float theta1 = qMax(undef, theta);
    for(const Range& r:invlin1.valued01()){
        //Range r = fce.range;
        if(auto s = r.intersect({theta,theta+glm::pi<float>()}))bottom.append({*s,lin1});
        //
        if(auto s = r.intersect({0.0,theta}))top.append({*s,lin1});
        if(auto s = r.intersect({theta+glm::pi<float>(),glm::two_pi<float>()}))top.append({*s,lin1});
    }
    for(const Range& r:invlin2.valued01()){
        //Range r = fce.range;
        if(auto s = r.intersect({theta,theta+glm::pi<float>()}))top.append({*s,lin2});
        //
        if(auto s = r.intersect({0.0,theta}))bottom.append({*s,lin2});
        if(auto s = r.intersect({theta+glm::pi<float>(),glm::two_pi<float>()}))bottom.append({*s,lin2});
    }
    for(const Range& r:invrat1.valued01()){
        //Range r = fce.range;
        if(auto s = r.intersect({0.0,theta0}))bottom.append({*s,rat1});
        if(auto s = r.intersect({theta1,theta0+glm::pi<float>()}))bottom.append({*s,rat1});
        if(auto s = r.intersect({theta1+glm::pi<float>(),2.0*glm::pi<float>()}))bottom.append({*s,rat1});
        //
        if(auto s = r.intersect({theta0,theta1}))top.append({*s,rat1});
        if(auto s = r.intersect({theta0+glm::pi<float>(),theta1+glm::pi<float>()}))top.append({*s,rat1});
    }
    for(const Range& r:invrat2.valued01()){
        //Range r = fce.range;
        if(auto s = r.intersect({0.0,theta0}))top.append({*s,rat2});
        if(auto s = r.intersect({theta1,theta0+glm::pi<float>()}))top.append({*s,rat2});
        if(auto s = r.intersect({theta1+glm::pi<float>(),2.0*glm::pi<float>()}))top.append({*s,rat2});
        //
        if(auto s = r.intersect({theta0,theta1}))bottom.append({*s,rat2});
        if(auto s = r.intersect({theta0+glm::pi<float>(),theta1+glm::pi<float>()}))bottom.append({*s,rat2});
    }
    Q_ASSERT(top.size()<=8);
    Q_ASSERT(bottom.size()<=8);
    //every .valued01() returns at most 3 intervals,
    //so top and bottom have at most 24 segments each,
    //hence it's all O(1), even with sorting
    //TODO: sort top and bottom?
}

/*QVector<BFPFace> RestrictedRegion::subregions()
{
    //top and bottom have at most 24 segments each,
    //with at least two shared region ends that's 23 segment endpoints for each
    //or 46 segment endpoints total to distribute between region ends,
    //that gives upped bound of 47 subregions
}*/

QVector<RestrictedRegion::Segment> RestrictedRegion::ceiling() const
{
    return top;
}

QVector<RestrictedRegion::Segment> RestrictedRegion::flooring() const
{
    return bottom;
}

QVector<RestrictedRegion::Segment> RestrictedRegion::ceilingAndFlooring() const
{
    QVector<RestrictedRegion::Segment> ret;
    ret.append(top);
    ret.append(bottom);
    return ret;
}

#include <complex>

float RestrictedRegion::solveCubic(double a, double b, double c, double d)
{
    if(d==0)return 0.0;
    std::complex<double> W = -2.0*b*b*b+9.0*a*b*c-27.0*a*a*d;
    std::complex<double> D = -b*b*c*c+4.0*a*c*c*c+4.0*b*b*b*d-18.0*a*b*c*d+27.0*a*a*d*d;
    std::complex<double> Q = W - a*std::sqrt(27.0*D);
    std::complex<double> w = {-0.5,std::sqrt(0.75)};
    std::complex<double> w2 = w*w;
    std::complex<double> R = std::exp(std::log(4.0*Q)/3.0);
    glm::tvec3<std::complex<double>> L = {{2.0*b*b-6.0*a*c,0.0},
                                          {-b,0.0},
                                          R};
    glm::tvec3<std::complex<double>> M = {3.0*a*R,
                                          {3.0*a,0.0},
                                          {6.0*a,0.0}};
    glm::tvec3<std::complex<double>> X = { L[0]/M[0],
                                           L[1]/M[1],
                                           L[2]/M[2]};
    glm::tmat3x3<std::complex<double>> mat = {w,1.0,w2,
                                           1.0,1.0,1.0,
                                           w2,1.0,w};
    glm::tvec3<std::complex<double>> x = mat*X;
    int i = 0;
    if(std::fabs(x[1].imag())<std::fabs(x[i].imag()))i=1;
    if(std::fabs(x[2].imag())<std::fabs(x[i].imag()))i=2;
    /*qDebug()<<fixed<<"("<<a<<","<<b<<","<<c<<","<<d<<")";
    qDebug()<<x[0].real()<<","<<x[1].real()<<","<<x[2].real();
    qDebug()<<"("<<x[i].real()<<","<<x[i].imag()<<")";
    qDebug()<<(a*x[i]*x[i]*x[i]+b*x[i]*x[i]+c*x[i]+d).real();
    qDebug()<<(a*x[i]*x[i]*x[i]+b*x[i]*x[i]+c*x[i]+d).imag();*/
    return x[i].real();
}

double cubic(glm::double4 c, double t){
    return ((c[0]*t+c[1])*t+c[2])*t+c[3];
}

double bisect(double a,double b, glm::double4 c){
    if(cubic(c,a)==0.0)return a;
    if(cubic(c,b)==0.0)return b;
    double mid = (a+b)*0.5;
    if(mid==a||mid==b){
        //qDebug()<<" a :"<< a <<cubic(c,a);
        //qDebug()<<"mid:"<<mid<<cubic(c,mid);
        //qDebug()<<" b :"<< b <<cubic(c,b);
        return mid;
    }
    double midv = cubic(c,mid);
    if(cubic(c,a)*midv<0.0){
        return bisect(a,mid,c);
    }else{
        return bisect(mid,b,c);
    }
}

double RestrictedRegion::bisectCubic(double a, double b, double c, double d)
{
    double step = 1.0;
    double infl = -b/(3.0*a);
    glm::double4 q = {a,b,c,d};
    double x;
    if(cubic(q,infl)*a<0.0){
        while(cubic(q,infl)*cubic(q,infl+step)>0.0){
            step*=2.0;
        }
        x = bisect(infl,infl+step,q);
    }else {
        while(cubic(q,infl-step)*cubic(q,infl)>0.0){
            step*=2.0;
        }
        x = bisect(infl-step,infl,q);
    }
    //qDebug()<<fixed<<"("<<a<<","<<b<<","<<c<<","<<d<<")";
    //qDebug()<<x;
    //qDebug()<<cubic(q,float(x));
    //qDebug()<<cubic(q,x);
    return x;
}

glm::double2x3 RestrictedRegion::splitDegenerateConic(glm::double3x3 C0)
{
    //assuming degenerate conic(c0 is symmetrical and rank 1)
    double discriminant = C0[1][0]*C0[1][0]-4.0*C0[0][0]*C0[1][1];
    qDebug()<<"discriminant"<<discriminant;
    if(discriminant<0)return{{0,0,2},{0,0,2}};
    //
    /*C0[0][1] = C0[1][0] = (C0[0][1]+C0[1][0])/2.0;
    C0[0][2] = C0[2][0] = (C0[0][2]+C0[2][0])/2.0;
    C0[1][2] = C0[2][1] = (C0[1][2]+C0[2][1])/2.0;*/
    //
    double a = C0[1][1]*C0[2][2]-C0[1][2]*C0[2][1];
    double b = C0[0][0]*C0[2][2]-C0[0][2]*C0[2][0];
    double c = C0[0][0]*C0[1][1]-C0[0][1]*C0[1][0];
    double d = C0[0][1]*C0[2][2]-C0[0][2]*C0[2][1];
    double e = C0[0][1]*C0[1][2]-C0[0][2]*C0[1][1];
    double f = C0[0][0]*C0[1][2]-C0[1][0]*C0[0][2];
    glm::double3x3 B = { a,d,e,
                        d,b,f,
                        e,f,c};
    int i = 0;
    if(std::fabs(B[i][i])<std::fabs(B[1][1]))i = 1;
    if(std::fabs(B[i][i])<std::fabs(B[2][2]))i = 2;
    //Q_ASSERT(B[i][i]<0.0);
    if(B[i][i]==0.0){
        B=C0;
    }else{
        glm::double3 p = B[i]/std::sqrt(std::fabs(B[i][i]));
        glm::double3x3 Mp = {0.0,-p[2],p[1],
                            p[2],0.0,-p[0],
                           -p[1],p[0],0.0};
        B = C0+Mp;
    }
    //
    int j=0;i=0;
    if(std::fabs(B[i][j])<fabs(B[0][1]))j = 1;
    if(std::fabs(B[i][j])<fabs(B[0][2]))j = 2;
    if(std::fabs(B[i][j])<fabs(B[1][0])){i = 1;j = 0;}
    if(std::fabs(B[i][j])<fabs(B[1][1])){i = 1;j = 1;}
    if(std::fabs(B[i][j])<fabs(B[1][2])){i = 1;j = 2;}
    if(std::fabs(B[i][j])<fabs(B[2][0])){i = 2;j = 0;}
    if(std::fabs(B[i][j])<fabs(B[2][1])){i = 2;j = 1;}
    if(std::fabs(B[i][j])<fabs(B[2][2])){i = 2;j = 2;}
    glm::double2x3 ret = {{B[i][0],B[i][1],B[i][2]},{B[0][j],B[1][j],B[2][j]}};
    glm::double3x3 outer = glm::outerProduct(ret[0],ret[1])+glm::outerProduct(ret[1],ret[0]);
    qDebug()<<"outer ratio:";
    for(int i=0;i<3;++i)
    qDebug()<<(outer[0][i]/C0[0][i])<<(outer[1][i]/C0[1][i])<<(outer[2][i]/C0[2][i]);
    return ret;
}

QVector<float> RestrictedRegion::intersectConicWithUnitCircle(glm::double3x3 B)
{
    //solve det(l*U+B)=0 for l, where U is unit circle
    glm::double3x3 U = {1,0,0,0,1,0,0,0,-1};
    double l = solveCubic(glm::determinant(U),
                        //B[2][2]-B[1][1]-B[0][0],
                         glm::determinant(glm::double3x3{U[0],U[1],B[2]})+glm::determinant(glm::double3x3{U[0],B[1],U[2]})+glm::determinant(glm::double3x3{B[0],U[1],U[2]}),
                        //B[1][1]*B[2][2]-B[1][2]*B[2][1]
                          //  +B[0][0]*B[2][2]-B[0][2]*B[2][0]
                            //    -B[0][0]*B[1][1]+B[0][1]*B[1][0],
                         glm::determinant(glm::double3x3{U[0],B[1],B[2]})+glm::determinant(glm::double3x3{B[0],U[1],B[2]})+glm::determinant(glm::double3x3{B[0],B[1],U[2]}),
                        glm::determinant(B));
    B[0][0]+=l;
    B[1][1]+=l;
    B[2][2]-=l;
    qDebug()<<"Bdet:"<<glm::determinant(B);
    glm::double2x3 gh = splitDegenerateConic(B);
    QVector<float> ret;
    if(auto g = solveLinearTrig(gh[0][2],gh[0][0],gh[0][1])){
        ret.append({g->x,g->y});
    }
    if(auto h = solveLinearTrig(gh[1][2],gh[1][0],gh[1][1])){
        ret.append({h->x,h->y});
    }
    return ret;
}

QVector<float> RestrictedRegion::solveQuadraticTrig(double x2, double y2,
                                                    double xy,
                                                    double x, double y,
                                                    double u)
{
    /*glm::double3x3 conicMatrix={ 2*x2, xy, x,
                                xy, 2*y2, y,
                                x,  y,  2*u};
    return intersectConicWithUnitCircle(conicMatrix);*/
    /*QVector<float> ret;
    ret=solveQuartic(   (x2-y2)*(x2-y2)+xy*xy,
                        2.0*x*(x2-y2)+2.0*y*xy,
                        2.0*(u+y2)*(x2-y2)+x*x+y*y-xy*xy,
                        2.0*(u*x+x*y2-y*xy),
                        u*u+2.0*u*y2-y*y);
    ret.erase(std::remove_if(ret.begin(),ret.end(),[](float t){
        return std::fabs(t)>1.0;
    }),ret.end());
    QVector<float> angles;
    for(float t:ret){
        t=acos(t);
        angles.append(t);
        angles.append(glm::two_pi<float>()-t);
    }
    return angles;*/
    QVector<float> ret;
    ret = solveQuartic( u-x+x2,
                        2.0*(y-xy),
                        2.0*(u-x2+2.0*y2),
                        2.0*(xy+y),
                        u+x+x2);
    QVector<float> angles;
    for(float t:ret){
        angles.append(flipToRange(0.0,glm::two_pi<float>(),2.0*std::atan(t)));
    }
    return angles;
}

std::optional<glm::float2> RestrictedRegion::solveLinearTrig(float A, float B, float C, float base)
{
    float eps = 10e-5;
    float hypot = std::hypot(B,C);
    if(hypot<=eps)return std::nullopt;
    float sine = -A/hypot;
    if(std::fabs(sine)>1.0f){
        return std::nullopt;
    }
    float arcsin = std::asin(sine);
    float arctan = std::atan2(B,C);
    float pi = glm::pi<float>();

    float z0 = arcsin-arctan;
    float z1 = pi-arcsin-arctan;
    z0=flipToRange(base,2*pi+base,z0);
    z1=flipToRange(base,2*pi+base,z1);
    if (z0<=z1)
        return std::make_optional(glm::float2{z0,z1});

    return std::make_optional(glm::float2{z1,z0});
}

QMap<int,int>RestrictedRegion::maxiter=QMap<int,int>();

QVector<float> RestrictedRegion::solveQuartic(double a4, double a3, double a2, double a1, double a0)
{
    QVector<float> ret;
    double u=a3/a4,v=a2/a4;
    double f0=a4;
    double b2=a4;
    double b1,b0;
    double c,d;
    int iter = 0;
    do{
        b1=a3-u*b2, b0=a2-u*b1-v*b2;
        c = a1-u*b0-v*b1, d = a0-v*b0;
        double g = b1 - u*f0, h = b0-v*f0;
        double det = v*g*g+h*(h-u*g);
        u = u-(-h*c+g*d)/det;
        v = v-(-g*v*c+(g*u-h)*d)/det;
        iter++;
        if(iter>=1000)break;
    }while(dot2({c,d},{c,d})>1.0e-10);
    maxiter[iter]++;
    if(iter>=1000)qDebug()<<"maxiter:"<<maxiter;
    //
    //solve two quadratics: [1,u,v] and [b2,b1,b0]
    double delta0 = u*u-4.0*v;
    if(delta0>=0.0){
        double s = std::sqrt(delta0);
        ret.append((-u-s)*0.5);
        ret.append((-u+s)*0.5);
    }
    double delta1 = b1*b1-4.0*b2*b0;
    if(delta1>=0.0){
        double s = std::sqrt(delta1);
        ret.append((-b1-s)*0.5/b2);
        ret.append((-b1+s)*0.5/b2);
    }
    return ret;
}

#include <functional>

QVector<float> RestrictedRegion::Segment::intersect(const RestrictedRegion::Segment &s)
{
    QVector<float> ret;
    auto intersection = r.intersect(s.r);
    if(!intersection)return ret;

    if(dcf.isTrigLinear()){
        if(s.dcf.isTrigLinear()){
            if(auto z = solveLinearTrig(dcf.A-s.dcf.A,dcf.B-s.dcf.B,dcf.C-s.dcf.C))
                ret.append({z->x,z->y});
        }else if(s.dcf.isTrigRational()){
            ret.append(solveQuadraticTrig(double(dcf.B)*s.dcf.D,
                                          double(dcf.C)*s.dcf.E,
                                          double(dcf.B)*s.dcf.E+double(dcf.C)*s.dcf.D,
                                          double(dcf.A)*s.dcf.D-double(s.dcf.B),
                                          double(dcf.A)*s.dcf.E-double(s.dcf.C),
                                          -double(s.dcf.A)));
        }
    }else if(dcf.isTrigRational()){
        if(s.dcf.isTrigLinear()){
            ret.append(solveQuadraticTrig(double(s.dcf.B)*dcf.D,
                                          double(s.dcf.C)*dcf.E,
                                          double(s.dcf.B)*dcf.E+double(s.dcf.C)*dcf.D,
                                          double(s.dcf.A)*dcf.D-double(dcf.B),
                                          double(s.dcf.A)*dcf.E-double(dcf.C),
                                          -double(dcf.A)));
        }else if(s.dcf.isTrigRational()){
            ret.append(solveQuadraticTrig(double(dcf.B)*s.dcf.D-double(s.dcf.B)*dcf.D,
                                          double(dcf.C)*s.dcf.E+double(s.dcf.C)*dcf.E,
                                          double(dcf.B)*s.dcf.E-double(s.dcf.B)*dcf.E+double(dcf.C)*s.dcf.D-double(s.dcf.C)*dcf.D,
                                          double(dcf.A)*s.dcf.D-double(s.dcf.A)*dcf.D,
                                          double(dcf.A)*s.dcf.E-double(s.dcf.A)*dcf.E,
                                          0.0));
        }
    }
    ret.erase(std::remove_if(ret.begin(),ret.end(),[&intersection](float t){
        return !(intersection->contains(t));
    }),ret.end());
    return ret;
}
