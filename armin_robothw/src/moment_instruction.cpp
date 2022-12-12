#include <string.h>
#include <stdlib.h>

#include <string>
#include <iostream>

#include "moment_instruction.h"

MomentInstructions::MomentInstructions() : executionMoment(0), instrType(type_velocity),
    _isScheduled(false)
{
}

MomentInstructions::~MomentInstructions()
{
}

MomentInstructions * MomentInstructions::fromLine(const char *ln)
{
    if (ln == nullptr)
        return nullptr;
    if (strlen(ln) == 0)
        return nullptr;

    char * line = strdup(ln);
    char * restor, *nxt;
    nxt = strtok_r(line, ";", &restor);
    if (nxt == nullptr) {
        free(line);
        return nullptr;
    }
    if ( nxt[0] == 'p') {
        free(line);
        return positionalFromLine(ln);
    }
    if ( nxt[0] == 'd') {
        free(line);
        return delayFromLine(ln);
    }
    if ( nxt[0] == 'v')
        nxt++;
    double exMomCandi = atof(nxt);

    nxt = strtok_r(NULL, ";", &restor);
    if (nxt == nullptr){
        free(line);
        return nullptr;
    }
    char * entries;
    entries = strtok_r(nxt, ",", &restor);
    std::list<std::pair<int, double> > candsL;
    while( entries != nullptr) {
        char * left, *right, *ress;
        left = strtok_r( entries, ":", &ress);
        right = strtok_r( nullptr, ":", &ress);
        if (left != nullptr && right != nullptr) {
            int lefV = atoi(left);
            double rv = atof ( right);
            candsL.push_back( { lefV, rv});
        }

        entries = strtok_r( nullptr, ",", &restor);
    }

    free (line);
    if ( candsL.size() > 0 && exMomCandi >= 0.) {
        auto retVal  = new MomentInstructions;
        retVal->executionMoment = exMomCandi;
        retVal->listOfMotorIDX_velocity = std::move( candsL);
        return retVal;
    }
    return nullptr;
}

MomentInstructions * MomentInstructions::positionalFromLine(const char *ln)
{
    char * line = strdup(ln);
    char * restor, *nxt;
    nxt = strtok_r(line, ";", &restor);
    // sanity check
    if ( nxt[0] != 'p') {
        free(line);
        return nullptr;
    }
    double exMomCandi = atof(nxt+ 1);
    nxt = strtok_r(NULL, ";", &restor);
    if (nxt == nullptr){
        free(line);
        return nullptr;
    }
    char * entries;
    entries = strtok_r(nxt, ",", &restor);
    std::list<MotorPositionSpeedEndSpeed > candsL;
    while( entries != nullptr) {
        char * idx, *position, *veloc, *eveloc, *ress;
        idx = strtok_r( entries, ":", &ress);
        position = strtok_r( nullptr, ":", &ress);
        veloc = strtok_r( nullptr, ":", &ress);
        eveloc = strtok_r( nullptr, ":", &ress);
        if (idx != nullptr && position != nullptr && veloc != nullptr) {
            int idxV = atoi(idx);
            int posV = atoi ( position);
            int velocV = atoi(veloc);
            int evelocV = 0;
            if (eveloc != nullptr) {
                evelocV = atoi(eveloc);
            }
            candsL.push_back( { idxV, posV, velocV, evelocV});
        }

        entries = strtok_r( nullptr, ",", &restor);
    }

    free (line);
    if ( candsL.size() > 0 && exMomCandi >= 0.) {
        auto retVal  = new MomentInstructions;
        retVal->executionMoment = exMomCandi;
        retVal->listMotorPositionSpeedEndSpeed = std::move( candsL);
        retVal->instrType = type_positional;
        return retVal;
    }
    return nullptr;
}

MomentInstructions * MomentInstructions::delayFromLine(const char *ln)
{
    char * line = strdup(ln);
    char * restor, *nxt;
    nxt = strtok_r(line, ";", &restor);
    // sanity check
    if ( nxt[0] != 'd') {
        free(line);
        return nullptr;
    }
    double exMomCandi = atof(nxt+ 1);
    nxt = strtok_r(NULL, ";", &restor);
    if (nxt == nullptr){
        free(line);
        return nullptr;
    }
    char * entries;
    entries = strtok_r(nxt, ",", &restor);
    double delayR = 0.;
    if (entries != nullptr) {
        delayR = atof(entries);
    }
    free (line);
    if ( delayR > 0. && exMomCandi >= 0.) {
        auto retVal  = new MomentInstructions;
        retVal->executionMoment = exMomCandi;
        retVal->delayDuration = delayR;
        retVal->instrType = type_delay;
        return retVal;
    }
    return nullptr;
}

bool MomentInstructions::operator<(const MomentInstructions &b)
{
    return executionMoment < b.executionMoment;
}

bool MomentInstructions::isDone(std::vector<bool> states) const
{
    int idx;
    auto j = std::begin(states);
    for( auto const & i: listMotorPositionSpeedEndSpeed) {
        if (i.idx > states.size() + 1 || i.idx < 1)
            continue;
        if ( !(*(j + i.idx -1)))
            return false;
    }
    return true;
}

void MomentInstructions::setScheduled()
{
    std::cout<<"Setting scheduled for object "<<this<<std::endl;
    _isScheduled = true;
}

