#include "AStarPlanner.h"
#include <queue>
#include <unordered_map>
#include <algorithm> // reverse

#include <iostream>

using namespace std;

ostream& operator<<(ostream& os, const Path& path)
{
    for (auto loc : path) {
        os << loc << " ";
    }
    return os;
}

Path AStarPlanner::make_path(const AStarNode* goal_node) const {
    Path path;
    const AStarNode* curr = goal_node;
    while (curr != nullptr) {
        path.push_back(curr->location);
        curr = curr->parent;
    }
    std::reverse(path.begin(),path.end());
    // cout << "something" << endl;
    return path;
}

bool AStarPlanner::check_visited(int location, unordered_map<pair<int, int>, AStarNode*, hash_pair> all_nodes) {
    for (auto n : all_nodes) {
        if (n.first.first == location) {
            return true;
        }
    }
    return false;
}

Path AStarPlanner::find_path(int agent_id, const list<Constraint>& constraints) {
    int start_location = ins.start_locations[agent_id];
    int goal_location = ins.goal_locations[agent_id];
    // Open list
    priority_queue<AStarNode*, vector<AStarNode*>, CompareAStarNode> open;


    // cout << "Agent: " << agent_id << " Constraints: " << constraints.size() << endl;
    // for (auto i : constraints) {
        // cout << get<0>(i) << " " << get<1>(i) << " " << get<2>(i) << " " << get<3>(i) << endl;
        // checked_constraints[i] = false;
    // }

    // Unordered map is an associative container that contains key-value pairs with unique keys.
    // The following unordered map is used for duplicate detection, where the key is the location of the node.
    // unordered_map<int, AStarNode*> all_nodes;
    // For Task 1, you need to replace the above line with
    unordered_map<pair<int, int>, AStarNode*, hash_pair> all_nodes;

    int h = ins.get_Manhattan_distance(start_location, goal_location); // h value for the root node
    auto root = new AStarNode(start_location, 0, h, nullptr);
    open.push(root);

    Path path;

    while (!open.empty()) {
        auto curr = open.top();
        // cout << "Agent: " << agent_id << " at location " << curr->location << " at time " << curr->timestep << endl;
        open.pop();

        // goal test
        if (curr->location == goal_location) {
            // adding a check for special cases where the agent can't stay at the goal location forever
            bool chk = true;
            int cur_time = curr->timestep;
            for (auto c_it = constraints.begin(); c_it != constraints.end(); ++c_it) {
                if (cur_time <= get<3>(*c_it) && checked_constraints.find(*c_it) == checked_constraints.end()) {
                    if (get<1>(*c_it) == curr->location && get<2>(*c_it) == -1) {
                        chk = false;
                    }
                }
            }
            
            if (chk) {
                path = make_path(curr);
                break;
            }
        }
        // checking for time out
        else if (curr->timestep >= ins.map_size() * ins.map_size()) {
            break;
        }

        // generate child nodes
        for (auto next_location : ins.get_adjacent_locations(curr->location)) {
            pair<int, int> loc_time_next = make_pair(next_location, curr->timestep + 1);
            auto it = all_nodes.find(loc_time_next);
            if (it == all_nodes.end()) {// the location has not been visited before
                int next_g = curr->g + 1;
                int next_h = ins.get_Manhattan_distance(next_location, goal_location);
                auto next = new AStarNode(next_location, next_g, next_h, curr);
                next->timestep = curr->timestep + 1;

                // checking constraints
                bool prune = false;
                
                for (auto c : constraints) {
                    int agent = get<0>(c);
                    int location_x = get<1>(c);
                    int location_y = get<2>(c);
                    int t = get<3>(c);

                    

                    // if it is a vertex constraint
                    if (location_y == -1) {
                        if (next_location == location_x && next->timestep == t && agent_id == agent) {
                            prune = true;
                            checked_constraints[c] = true;
                        }
                    } 
                    // if it is not a vertex constraint
                    else {
                        if (curr->location == location_x && next_location == location_y 
                            && curr->timestep == t - 1 && next->timestep == t 
                            && agent_id == agent) {
                                prune = true;
                                checked_constraints[c] = true;
                        }
                    }
                }

                if (!prune) {
                    // cout << "Agent: " << agent_id << " going from " << curr->location << " to " << next_location << " at time " << curr->timestep << endl;
                    open.push(next);
                    all_nodes[loc_time_next] = next;
                }
            }
            // Note that if the location has been visited before,
            // next_g + next_h must be greater than or equal to the f value of the existing node,
            // because we are searching on a 4-neighbor grid with uniform-cost edges.
            // So we don't need to update the existing node.
        }
        // Taking a wait action
        pair<int, int> loc_time_wait = make_pair(curr->location, curr->timestep + 1);
        auto wait = new AStarNode(curr->location, curr->g, curr->h, curr->parent);
        wait->timestep = curr->timestep + 1;
        all_nodes[loc_time_wait] = wait;
    }
    // release memory
    for (auto n : all_nodes)
        delete n.second;

    return path;
}