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
