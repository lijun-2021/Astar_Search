#include <iostream>
#include <thread>
#include "input.h"
#include "node.h"
#include "petri.h"

int main(int argc, char* argv[]) {
	/* 读数据 */
	std::vector<int> m0 = readVector<int>(kInitMark);
	std::vector<int> d0 = readVector<int>(kTimePath);
	std::vector<int> goals = readVector<int>(kGoalPlace);
	std::vector<int> goal_marking = readVector<int>(kGoalMarking);
	std::vector<int> goal_vector = getGoalMark(goal_marking, goals, m0.size());
	std::vector<vector<int>> pre = readMatrix(kPrePath, m0.size());
	std::vector<vector<int>> post = readMatrix(kPostPath, m0.size());

	// Petri网模型
	PetriNet petri(m0, d0, pre, post, goal_vector, 1);
	// A*搜索
	petri.AstarSearch(); 
	// 集束搜索
	//petri.BeamSearch(60);
	PetriNet::infoDisplay(&petri);

	//system("pause");

	return 0;
}
