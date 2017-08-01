#include "restrictedregion.h"
#include "misc/misc.h"

RestrictedRegion::RestrictedRegion(Contact c, QLineF obj, QLineF obs)
{
    Contact ev1{obj,obs.p1(),Contact::EdgeVertex}, ev2{obj,obs.p2(),Contact::EdgeVertex};
    Contact ve1{obs,obj.p1(),Contact::VertexEdge}, ve2{obs,obj.p2(),Contact::VertexEdge};
    ABF_ASSERT(!(ev1==c || ev2==c || ve1==c || ve2==c));
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
    ABF_ASSERT(top.size()<=9);
    ABF_ASSERT(bottom.size()<=9);
    //every .valued01() returns at most 3 intervals,
    //so top and bottom have at most 24 segments each,
    //hence it's all O(1), even with sorting
    //TODO: sort top and bottom?
    top = sortByRangeStart(top);
    bottom = sortByRangeStart(bottom);
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

QVector<RestrictedRegion::Segment> RestrictedRegion::sortByRangeStart
(const QVector<RestrictedRegion::Segment> &segs)
{
    QVector<RestrictedRegion::Segment> ret;
    //O(size^2), but size<=24 (and usually <=8) so it's O(1)
    QVector<float> starts,ends;
    QVector<int> nextids=QVector<int>(segs.size(),-1);
    QVector<int> previds=QVector<int>(segs.size(),-1);
    QVector<int> startids;
    for(int i=0;i<segs.size();++i){
        for(int j=0;j<ends.size();++j){
            if(segs[i].r.begin==ends[j])previds[i]=j;
            if(segs[i].r.end==starts[j])previds[j]=i;
        }
        starts.append(segs[i].r.begin);
        ends.append(segs[i].r.end);
    }
    for(int i=0;i<nextids.size();++i){
        if(previds[i]!=-1)nextids[previds[i]]=i;
        else startids.append(i);
    }
    ABF_ASSERT(startids.size()<=3);
    if(startids.size()>=2){
        if(starts[startids[0]]>starts[startids[1]])
            std::swap(startids[0],startids[1]);
    }
    if(startids.size()==3){
        if(starts[startids[1]]>starts[startids[2]])
            std::swap(startids[1],startids[2]);
        if(starts[startids[0]]>starts[startids[1]])
            std::swap(startids[0],startids[1]);
    }
    int j=0;
    int id = startids[j];
    for(int i=0;i<nextids.size();++i){
        if(id==-1)id = startids[++j];
        ret.append(segs[id]);
        id=nextids[id];
    }
    for(int i=1;i<ret.size();++i){
        ABF_ASSERT(ret[i-1].r.begin<ret[i].r.begin);
    }
    return ret;
}

QVector<float> RestrictedRegion::solveQuadraticTrig(double x2, double y2,
                                                    double xy,
                                                    double x, double y,
                                                    double u)
{
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
