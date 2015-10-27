/*
	Aseba - an event-based framework for distributed robot control
	Copyright (C) 2007--2015:
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

#ifndef ASEBA_HTTP_DASHEL_TARGET
#define ASEBA_HTTP_DASHEL_TARGET

#include <map>
#include <string>
#include <set>
#include <dashel/dashel.h>
#include "../../common/msg/descriptions-manager.h"
#include "AeslProgram.h"

namespace Aseba { namespace Http
{
	class HttpInterface; // foward declaration
	class HttpRequest; // foward declaration
	class DashelHttpRequest; // foward declaration

	class HttpDashelTarget : public DescriptionsManager
	{
		public:
			struct Node {
				unsigned localId;
				unsigned globalId;
				std::string name;
				VariablesMap variablesMap;
				std::map< unsigned, std::set< std::pair<Dashel::Stream *, DashelHttpRequest *> > > pendingVariables;
			};

			HttpDashelTarget(HttpInterface *interface, const std::string& address, Dashel::Stream *stream);
			virtual ~HttpDashelTarget();

			virtual bool sendEvent(const AeslProgram& program, const std::vector<std::string>& args);
			virtual bool sendGetVariables(unsigned globalNodeId, const std::vector<std::string>& args, HttpRequest *request);
			virtual bool sendSetVariable(unsigned globalNodeId, const std::vector<std::string>& args);
			virtual bool compileAndRunCode(unsigned globalNodeId, const AeslProgram& program, const std::string& code, std::string& errorString);

			virtual bool removePendingVariable(unsigned globalNodeId, unsigned start);

			virtual std::set<const Node *> getNodesByName(const std::string& name) const;
			virtual const Node *getNodeById(unsigned globalNodeId) const;
			virtual const Node *getNodeByLocalId(unsigned localNodeId) const;

			virtual const std::string& getAddress() const { return address; }
			virtual Dashel::Stream *getStream() { return stream; }
			virtual const std::map<unsigned, Node>& getNodes() const { return nodes; }

		protected:
			virtual bool getVariableInfo(const Node& node, const std::string& variableName, unsigned& position, unsigned& size);
			virtual void nodeDescriptionReceived(unsigned localNodeId);

		private:
			HttpInterface *interface;
			std::string address;
			Dashel::Stream *stream;

			std::map<unsigned, Node> nodes;
			std::map<unsigned, unsigned> globalIds;
	};
} }

#endif
