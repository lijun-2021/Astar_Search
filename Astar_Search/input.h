#pragma once
/* 原型 */
constexpr auto kInitMark = "./data/111/m0.txt"; 
constexpr auto kGoalMarking = "./data/111/goalMarking.txt";
constexpr auto kPrePath = "./data/Basic/pre.txt";
constexpr auto kPostPath = "./data/Basic/post.txt";
constexpr auto kTimePath = "./data/Basic/delay.txt";
//constexpr auto kTimePath = "./data/basic/delay_Huangbo.txt";
constexpr auto kGoalPlace = "./data/Basic/GoalPlace.txt";

/* 原型 + 死锁控制器 */
//constexpr auto kInitMark = "./data/controller/111/m0.txt";
//constexpr auto kGoalMarking = "./data/111/GoalMarking.txt";
//
//constexpr auto kPrePath = "./data/controller/pre.txt";
//constexpr auto kPostPath = "./data/controller/post.txt";
//constexpr auto kTimePath = "./data/controller/delay.txt";
//constexpr auto kGoalPlace = "./data/basic/GoalPlace.txt";

/* 神经网络 */
constexpr auto kNetWork = "C:/Users/Public/model/Model-6.pb";

#include <iostream>
#include <fstream>
#include <vector>
#include <string>

using std::vector;

template<class T>
/* 读取初始标识、延时 */
vector<T> readVector(const char* path) {
	T num;
	vector<T> result;
	std::ifstream file(path);
	if (!file.is_open()) {
		std::cout << "fail to oepn file :" << path << std::endl;
		return {};
	}
	while (file >> num)
		result.push_back(num);
	file.close();
	return result;
}

/* 读取关联矩阵 */
vector<vector<int>> readMatrix(const char* path, int m) {
	vector<vector<int>> matrix;
	std::ifstream file(path, std::ios::in);
	if (!file.is_open()) {
		std::cout << "can not open file:" << path << std::endl;
		exit(1);
	}
	int num = 0;
	vector<int> ans;
	while (file >> num) {
		ans.push_back(num);
	}
	int n = ans.size() / m;
	matrix.resize(m, vector<int>(n, 0));
	for (int i = 0; i < m; ++i)
		for (int j = 0; j < n; ++j)
			matrix[i][j] = ans[i*n + j];
	file.close();
	return matrix;
}

/* 获取目标节点标识 */
vector<int> getGoalMark(vector<int>& goalMarking, vector<int>& goalPlace, int nums) {
	vector<int> ans(nums, 0);
	if (goalMarking.size() != goalPlace.size()) {
		std::cout << "The dimension of goal marking and goal place is not consistent!" << std::endl;
		exit(-1);
	}
	for (int i = 0; i < goalPlace.size(); ++i) {
		ans[goalPlace[i]] = goalMarking[i];
	}
	return ans;
}
