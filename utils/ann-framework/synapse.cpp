/*****************************************************************************
 *  Copyright (C) 2012 by Timo Nachstedt                                     *
 *                                                                           *
 *  This program is free software: you can redistribute it and/or modify     *
 *  it under the terms of the GNU General Public License as published by     *
 *  the Free Software Foundation, either version 3 of the License, or        *
 *  (at your option) any later version.                                      *
 *                                                                           *
 *  This program is distributed in the hope that it will be useful,          *
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of           *
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the            *
 *  GNU General Public License for more details.                             *
 *                                                                           *
 *  You should have received a copy of the GNU General Public License        *
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.    *
 *                                                                           *
 ****************************************************************************/


#include "synapse.h"

#include "neuron.h"
#include "ann.h"

Synapse::Synapse(Neuron * const apost, Neuron * const apre)
: pre(apre), post(apost)
{
    pre->addSynapseOut(this);
    post->addSynapseIn(this);
    weight = 0;
}

Synapse::~Synapse()
{
    pre->removeSynapseOut(this);
    post->removeSynapseIn(this);
}

Neuron* Synapse::getPost() const
{
    return post;
}

Neuron* Synapse::getPre() const
{
    return pre;
}

const double& Synapse::getWeight() const
{
    return weight;
}

void Synapse::setWeight(const double & aweight)
{
    weight = aweight;
}
