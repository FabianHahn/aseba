/*
	Playground - An active arena to learn multi-robots programming
	Copyright (C) 1999--2013:
		Stephane Magnenat <stephane at magnenat dot net>
		(http://stephane.magnenat.net)
	3D models
	Copyright (C) 2008:
		Basilio Noris
	Aseba - an event-based framework for distributed robot control
	Copyright (C) 2007--2013:
		Stephane Magnenat <stephane at magnenat dot net>
		(http://stephane.magnenat.net)
		and other contributors, see authors.txt for details
	
	This program is free software: you can redistribute it and/or modify
	it under the terms of the GNU Lesser General Public License as published
	by the Free Software Foundation, version 3 of the License.
	
	This program is distributed in the hope that it will be useful,
	but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
	GNU Lesser General Public License for more details.
	
	You should have received a copy of the GNU Lesser General Public License
	along with this program. If not, see <http://www.gnu.org/licenses/>.
*/


#include "PlaygroundViewer.h"
#include "Parameters.h"
#include "EPuck.h"

namespace Enki
{
	static PlaygroundViewer* playgroundViewer = 0;

	PlaygroundViewer::PlaygroundViewer(World* world) : ViewerWidget(world), energyPool(INITIAL_POOL_ENERGY)
	{
		font.setPixelSize(16);
		#if QT_VERSION >= 0x040400
		font.setLetterSpacing(QFont::PercentageSpacing, 130);
		#elif (!defined(_MSC_VER))
		#warning "Some feature have been disabled because you are using Qt < 4.4.0 !"
		#endif
		if (playgroundViewer)
			abort();
		playgroundViewer = this;
	}
	
	PlaygroundViewer::~PlaygroundViewer()
	{
		
	}
	
	World* PlaygroundViewer::getWorld() const
	{
		return world;
	}
	
	PlaygroundViewer* PlaygroundViewer::getInstance()
	{
		return playgroundViewer;
	}
	
	void PlaygroundViewer::renderObjectsTypesHook()
	{
		managedObjectsAliases[&typeid(AsebaFeedableEPuck)] = &typeid(EPuck);
		// TODO: add display of Thymio 2 in EnkiViewer, then add alias here
	}
	
	void PlaygroundViewer::sceneCompletedHook()
	{
		// create a map with names and scores
		//qglColor(QColor::fromRgbF(0, 0.38 ,0.61));
		qglColor(Qt::black);
		
		// TODO: clean-up that
		int i = 0;
		QString scoreString("Id.: E./Score. - ");
		int totalScore = 0;
		for (World::ObjectsIterator it = world->objects.begin(); it != world->objects.end(); ++it)
		{
			AsebaFeedableEPuck *epuck = dynamic_cast<AsebaFeedableEPuck*>(*it);
			if (epuck)
			{
				totalScore += (int)epuck->score;
				if (i != 0)
					scoreString += " - ";
				scoreString += QString("%0: %1/%2").arg(epuck->vm.nodeId).arg(epuck->variables.energy).arg((int)epuck->score);
				renderText(epuck->pos.x, epuck->pos.y, 10, QString("%0").arg(epuck->vm.nodeId), font);
				i++;
			}
		}
		
		renderText(16, 22, scoreString, font);
		
		renderText(16, 42, QString("E. in pool: %0 - total score: %1").arg(energyPool).arg(totalScore), font);
	}
} // Enki
