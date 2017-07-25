#include "doublecontactfunction.h"

#include "misc/misc.h"

DoubleContactFunction::DoubleContactFunction(){}

DoubleContactFunction::DoubleContactFunction(Contact original, Contact drawing)
{
    originalContactType = original.type;
    drawingContactType = drawing.type;
    H = original.edge;
    J = original.vertex;
    G = drawing.edge;
    K = drawing.vertex;
    if (originalContactType==Contact::VertexEdge &&
            drawingContactType==Contact::VertexEdge){
        D=cross2(G.p2()-G.p1(),H.p2()-H.p1());
        A=cross2(G.p2()-G.p1(),G.p1()-H.p1())/D;
        B=cross2(G.p2()-G.p1(),J-K)/D;
        C=dot2(G.p2()-G.p1(),J-K)/D;
    }
    else if(originalContactType==Contact::VertexEdge &&
            drawingContactType==Contact::EdgeVertex){
        A=cross2(J-G.p1(),G.p2()-G.p1());
        B=cross2(K-H.p1(),G.p2()-G.p1());
        C=dot2(K-H.p1(),G.p2()-G.p1());
        D=cross2(H.p2()-H.p1(),G.p2()-G.p1());
        E=dot2(H.p2()-H.p1(),G.p2()-G.p1());
    }
    else if(originalContactType==Contact::EdgeVertex &&
            drawingContactType==Contact::VertexEdge){
        A=cross2(G.p2()-G.p1(),J-G.p1());
        B=cross2(G.p2()-G.p1(),K-H.p1());
        C=dot2(G.p2()-G.p1(),K-H.p1());
        D=cross2(G.p2()-G.p1(),H.p2()-H.p1());
        E=dot2(G.p2()-G.p1(),H.p2()-H.p1());
    }
    else if(originalContactType==Contact::EdgeVertex &&
            drawingContactType==Contact::EdgeVertex){
        D=cross2(H.p2()-H.p1(),G.p2()-G.p1());
        A=cross2(G.p1()-H.p1(),G.p2()-G.p1())/D;
        B=cross2(J-K,G.p2()-G.p1())/D;
        C=dot2(J-K,G.p2()-G.p1())/D;
    }
    computeZeroes();
    computeOnes();
}

bool DoubleContactFunction::isTrigLinear() const
{
    return (originalContactType==drawingContactType)
            && (originalContactType!=Contact::NoContact);
}

bool DoubleContactFunction::isTrigRational() const
{
    return (originalContactType!=drawingContactType)
            && (originalContactType!=Contact::NoContact)
            && (drawingContactType!=Contact::NoContact);
}

bool DoubleContactFunction::isValid() const
{
    return (drawingContactType!=Contact::NoContact)
            && (originalContactType!=Contact::NoContact);
}

float DoubleContactFunction::operator()(float theta) const {
    if (isTrigLinear()){
        return A+B*cos(theta)+C*sin(theta);
    }
    else if(isTrigRational()){
        return (A+B*cos(theta)+C*sin(theta))
                /(D*cos(theta)+E*sin(theta));
    }
    Q_UNREACHABLE();
}

float DoubleContactFunction::atZero() const {
    if (isTrigLinear()){
        return A+B;
    }
    else if(isTrigRational()){
        return (A+B)/(D);
    }
    Q_UNREACHABLE();
}

std::optional<float> DoubleContactFunction::undefinedValue() const {
    if(originalContactType==drawingContactType){
        //A+B*cos+C*sin version; always defined
        return std::nullopt;
    }else{
        //return theta when D*cos+E*sin==0
        float theta = -std::atan2(D,E);
        float pi = glm::pi<float>();
        //but in range in [0,pi)
        theta = flipToRange(0,2*pi,theta);
        return std::make_optional(theta);

        //TODO: unless A+B*cos+C*sin==0 as well
        //      then what?
    }
}

std::optional<glm::float2> DoubleContactFunction::zeroes() const
{
    return borderValues[0];
}

std::optional<glm::float2> DoubleContactFunction::ones() const
{
    return borderValues[1];
}

bool DoubleContactFunction::hasParallelMovement() const
{
    float eps = 10e-5;
    if(this->isTrigRational()){
        //TODO: check eps correctness
        return glm::abs(glm::abs(A)*H.length() - glm::abs(B)*G.length())
                < eps * H.length() * G.length();
    } else if(this->isTrigLinear()) {
        //TODO: check eps correctness
        return (glm::abs(cross2(H,G)) < eps)
             && QLineF(J,K).length()*G.length() >= cross2(G,{G.p1(),H.p1()});
    }
    Q_UNREACHABLE();
}

QVector<float> DoubleContactFunction::parallelMovementAngle() const
{
    QVector<float> ret;
    float eps = 10e-5;
    //
    if(!hasParallelMovement()){
        //return std::nullopt;
        return ret;
    }
    if(this->isTrigRational()){
        float arctan = std::atan2(cross2(G,H),dot2(G,H));
        ret.append(arctan);
        return ret;
    } else if(this->isTrigLinear()) {
        float sine = cross2(G,{G.p1(),H.p1()})/(QLineF(J,K).length()*G.length());
        float arcsin = std::asin(sine);
        float arctan = std::atan2(cross2(G,{K,J}),dot2(G,{K,J}));
        float pi = glm::pi<float>();
        if(std::fabs(sine)>1.0f-eps){
            ret.append(flipToRange(0,2*pi,arctan));
            return ret;
        }
        //solutions are distiguishable enough
        ret.append(flipToRange(0,2*pi,arcsin-arctan));
        ret.append(flipToRange(0,2*pi,pi-arcsin-arctan));
        return ret;
    }
    return ret;
}

QVector<Range> DoubleContactFunction::valued01() const
{
    QVector<Range> ret;
    if(this->isTrigRational()){
        //qDebug()<<"rat";
        float M = std::hypot(D,E);
        float T = (A*M+B*E-C*D);//same sign as (A*M+B*E-C*D)*0.5/(M*M);
        float S = (A*M+C*D-B*E);//same sign as (A*M+C*D-B*E)*0.5/(M*M);
        if(T * S >= 0.0){
            float c = (B*D+C*E)*2.0;//scaled as T&S from (B*D+C*E)/(M*M);
            if(c+2*sqrt(T*S)<0.0){//local min below 0
                //qDebug()<<"local min below 0";
                auto outs = *ones();
                auto ins = *zeroes();
                ret.append({outs.x,ins.x});
                ret.append({ins.y,outs.y});
            }else if(c-2*sqrt(T*S)>2.0*M*M){//local max above 1
                //qDebug()<<"local max above 1";
                auto outs = *zeroes();
                auto ins = *ones();
                ret.append({outs.x,ins.x});
                ret.append({ins.y,outs.y});
            }else{//intervals between ones and between zeroes are disjoint
                //qDebug()<<"separate";
                if(auto bounds = ones()){
                    //qDebug()<<"ones";
                    ret.append({bounds->x,bounds->y});
                }
                if(auto bounds = zeroes()){
                    //qDebug()<<"zeroes";
                    ret.append({bounds->x,bounds->y});
                }
            }
        }else if(S>0.0){//T<0.0
            //qDebug()<<"strictly increasing";
            //strictly increasing on every defined interval
            auto starts = *zeroes();
            auto ends = *ones();
            ret.append({starts.x,ends.x});
            ret.append({starts.y,ends.y});
        } else {//S<0.0, T>0.0
            //qDebug()<<"strictly decreasing";
            //strictly decreasing on every defined interval
            auto starts = *ones();
            auto ends = *zeroes();
            ret.append({starts.x,ends.x});
            ret.append({starts.y,ends.y});
        }
    }else if(this->isTrigLinear()){
        //qDebug()<<"lin";
        if(auto outs=ones()){
            //qDebug()<<"ones";
            if(auto ins=zeroes()){
                //qDebug()<<"zeroes";
                ret.append({outs->x,ins->x});
                ret.append({ins->y,outs->y});
            }else{
                ret.append({outs->x,outs->y});
            }
        }else{
            if(auto ins=zeroes()){
                //qDebug()<<"zeroes";
                /*ret.append({(ins->y+ins->x)*0.5-glm::pi<float>(),
                            ins->x});
                ret.append({ins->y,
                            (ins->y+ins->x)*0.5+glm::pi<float>()});*/
                ret.append({ins->y,ins->x+glm::two_pi<float>()});
            }else{
                if(atZero()<=1.0 && atZero()>=0.0){
                    //qDebug()<<"inside";
                    ret.append({0.0,glm::two_pi<float>()});
                }else{
                    //qDebug()<<"outside";
                }
            }
        }
    }
    //if(auto r=zeroes()){qDebug()<<"zeroes:"<<r->x<<","<<r->y;}
    //if(auto r=ones()){qDebug()<<"ones:"<<r->x<<","<<r->y;}
    //normalize to [0,2pi]?
    QVector<Range> normalized;
    float twopi = glm::two_pi<float>();
    for(Range r : ret){
        if(r.begin>=twopi){
            r={r.begin-twopi,r.end-twopi};
        }
        if(r.end>twopi){
            normalized.append({r.begin,twopi});
            normalized.append({0.0,r.end-twopi});
        }else{
            normalized.append(r);
        }
    }
    Q_ASSERT(normalized.size()<=3);
    return normalized;
}

void DoubleContactFunction::computeZeroes()
{
    if(isTrigRational()){
        //TODO: what if for some nominator solution
        //      denominator dissapears as well
        //      i.e. D*cos+E*sin==0
        //do nothing? ranges must break there anyway
    }
    float eps = 10e-5;
    float pi = glm::pi<float>();

    if(isTrigLinear()){
        //TODO: check eps correctness
        if(std::fabs(D)<eps){//parallel
            borderValues[0] = std::nullopt;
            return;
        }
    }

    //return solutions of f(theta)==0
    //always equivalent with A+B*cos+C*sin==0
    float base = isTrigLinear()?
                pi-atan2(C,-B):
                pi-atan2(D,E);
    borderValues[0] = solveLinearTrig(A,B,C,base);
}

void DoubleContactFunction::computeOnes()
{
    if(isTrigRational()){
        //TODO: what if for some nominator solution
        //      denominator dissapears as well
        //      i.e. D*cos+E*sin==0
    }
    float eps = 10e-5;
    float pi = glm::pi<float>();
    //return solutions of f(theta)==1
    //equivalent to:
    //(A-1)+B*cos+C*sin==0 - for linear version
    //A+(B-D)*cos+(C-E)*sin==0 - for rational version
    //solve it same as 'zeroes', but with local values
    if(isTrigLinear()){
        //TODO: check eps correctness
        if(std::fabs(D)<eps){//parallel
            borderValues[1] = std::nullopt;
            return;
        }
        borderValues[1] = solveLinearTrig(A-1,B,C,pi-atan2(C,-B));
        return;
    }
    if(isTrigRational()){
        borderValues[1] = solveLinearTrig(A,B-D,C-E,pi-atan2(D,E));
        return;
    }
    Q_UNREACHABLE();
}

std::optional<glm::float2> DoubleContactFunction::solveLinearTrig(float A, float B, float C, float base)
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
