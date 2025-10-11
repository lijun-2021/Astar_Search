#pragma once
#include <algorithm>
#include <numeric>
#include <ctime>
#include <map>
#include <list>
#include <queue>
#include <tuple>
#include <string>
#include <unordered_set>
#include "node.h"
#include "tensor.h"
#include <fstream>

/* 资源个数 */
#define RESOURCE_COUNT 7
/* 机械臂资源个数 */
#define ROBOT_COUNT 3
/* 启发式 */
#define SELETE_H 5

using std::string;
using std::list;

static std::unordered_set<int> ignore_m = {
	26,	27,	28
}, ignore_v = {
	0,	4,	13,
	19,	20,	21, 22, 23, 24, 25,
	26, 27, 28
};

/* A Star */
class AStar {
public:
	bool operator()(const ptrNode n1, const ptrNode n2) {
		return (n1->g_ + n1->h_) > (n2->g_ + n2->h_);
	}
};

class PetriNet {
public:
	/* 库所个数 */
	int num_place_;
	/* 变迁个数 */
	int num_transition_;
	/* 最短加工时间 */
	int g_min_;
	/* 扩展总结点数 */
	unsigned int total_;
	/* 节点池 */
	NodePool pool_;
	/* 根节点 */
	ptrNode root_;
	/* 前置矩阵转置 */
	vector<vector<int>> Tpre_;
	/* 后置矩阵转置 */
	vector<vector<int>> Tpost_;
	/* 关联矩阵转置 */
	vector<vector<int>> C;
	/* 初始标识 */
	vector<int> m0_;
	/* 延时 */
	vector<int> delay_;
	/* 目标标识 */
	vector<int> goal_;
	/* 存储新节点 open表 */
	std::priority_queue<ptrNode, vector<ptrNode>, AStar> open_list_;
	/* 存储扩展过的节点 close + open表 */
	std::map<string, list<ptrNode>> entire_list_;
	/* 神经网络启发式 */
	//std::shared_ptr<Tensor> nn_;
	/* 资源库所 */
	vector<int> resources_;
	/* 目标节点表 */
	vector<ptrNode> goal_nodes_;
	/* 四条路径的动作库所 */
	vector<vector<int>> paths_;
	/* 资源输出变迁的前置库所和后置库所 */
	vector<std::pair<vector<int>,vector<int>>> res_paths_;
	/* 占用资源库所的操作库所 */
	vector<std::unordered_set<int>> work_by_resource_;

	/* 初始化参数 */
	PetriNet(vector<int>& m, vector<int>& d, vector<vector<int>>& p, vector<vector<int>>& q, vector<int>& goal, unsigned int num_threads = 1) :
		pool_(10000000), m0_(m), delay_(d), goal_(goal), /*nn_(std::make_shared<Tensor>()),*/ g_min_(INT_MAX), total_(1) {
		if (p.size() == 0 || m0_.size() == 0 || delay_.size() == 0 || goal_.size() == 0 || q.size() == 0) {
			printf("input files error!\n");
			exit(-1);
		}

		num_place_ = m.size();
		num_transition_ = p[0].size();

		Tpre_.resize(num_transition_, vector<int>(num_place_, 0));
		Tpost_.resize(num_transition_, vector<int>(num_place_, 0));
		C.resize(num_transition_, vector<int>(num_place_, 0));

		for (int i = 0; i < num_place_; ++i) {
			int num_of_place = 0;
			for (int j = 0; j < num_transition_; ++j) {
				Tpre_[j][i] = p[i][j];
				Tpost_[j][i] = q[i][j];
				C[j][i] = q[i][j] - p[i][j];
				if (p[i][j] != 0) {
					++num_of_place;
				}
			}
		}

		root_ = pool_.getNode();
		for (int i = 0; i < num_place_; ++i) {
			if (m0_[i] != 0)
				root_->state_.emplace_back(Place(m0_[i], i, { 0, 0 }));
		}

		//open_list_.push(root_);
		root_->is_open_ = true;
		std::list<ptrNode> temp = { root_ };
		entire_list_.emplace(root_->toString(), std::move(temp));  // move移动资源，原地址存放的资源不存在

		/* 直线路径包含库所 */
		paths_.assign(4, {});
		/* 路径1中的动作库所 */
		paths_[0] = { 0, 1, 2, 3, 26 };
		/* 路径2中的动作库所 */
		paths_[1] = { 4, 5, 10, 11, 12, 9, 27 };
		/* 路径3中的动作库所 */
		paths_[2] = { 4, 5, 6, 7, 8, 9, 27 };
		/* 路径4中的动作库所 */
		paths_[3] = { 13, 14, 15, 16, 17, 18, 28 };

		/* first：资源前置库所 second：资源后置库所 */
		res_paths_.resize(7, { {}, {} });
		/* R1 */
		res_paths_[0].first = { 4, 17 }; res_paths_[0].second = { 5, 18 };
		/* R2 */
		res_paths_[1].first = { 0, 2, 6, 10, 15 }; res_paths_[1].second = { 1, 3, 7, 11, 16 };
		/* R3 */
		res_paths_[2].first = { 8, 12, 13 }; res_paths_[2].second = { 9, 9, 14 };
		/* M1 */
		res_paths_[3].first = { 5 }; res_paths_[3].second = { 6 };
		/* M2 */
		res_paths_[4].first = { 1, 7 }; res_paths_[4].second = { 2, 8 };
		/* M3 */
		res_paths_[5].first = { 5, 16 }; res_paths_[5].second = { 10, 17 };
		/* M4 */
		res_paths_[6].first = { 11, 14 }; res_paths_[6].second = { 12, 15 };
		/* 资源库所id */
		resources_ = { 19, 20, 21, 22, 23, 24, 25 };

		/* 占用资源库所的操作库所 */
		work_by_resource_.push_back({ 5, 18 });
		work_by_resource_.push_back({ 1, 3, 7, 11, 16 });
		work_by_resource_.push_back({ 9, 14 });
		work_by_resource_.push_back({ 6 });
		work_by_resource_.push_back({ 2, 8 });
		work_by_resource_.push_back({ 10, 17 });
		work_by_resource_.push_back({ 12, 15 });

		return;
	}

	/* 查找库所所在加工路径 */
	std::pair<int, int> findPosition(int id) const {
		for (int i = 0; i < paths_.size(); ++i) {
			for (int j = 0; j < paths_[i].size(); ++j) {
				if (paths_[i][j] == id) {
					return std::make_pair(i, j);
				}
			}
		}
		return std::make_pair(-1, -1);
	}

	/* 启发式(1) */
	float heuristicsOne(ptrNode& node, int idle_time = 0, int ER = 11) {
		// ER:资源个数  time:总时间
		float time = 0;

		// 累加各个路径上的工件到达各自目标库所的时间
		for (auto& p : node->state_) {
			// 查询结果：存在返回(路径i, 库所j) 不存在返回(-1, -1)
			std::pair<int, int> res = findPosition(p.row_);
			if (res.first == -1) {
				continue;
			}
			// 累加当前库所还需等待的时间
			std::for_each(p.v_.begin(), p.v_.begin() + std::min(2, (int)p.tokens_), [&](int16_t& t) noexcept {
				time += std::max(0, delay_[p.row_] - t);
				});

			// 累加后续库所还需等待的时间
			for (int i = res.second + 1; i < paths_[res.first].size(); ++i) {
				time += delay_[paths_[res.first][i]] * p.tokens_;
			}
		}
		return (time + idle_time) / ER;
	}

	/* 启发式(2) */
	float heuristicsTwo(ptrNode& node, 
		vector<int> vec = vector<int>(RESOURCE_COUNT, 1)) {
		// 总空闲时间
		int idle_time = 0;
		// OT RT
		vector<std::pair<vector<int>, vector<int>>> OTs_and_RTs(RESOURCE_COUNT);

		for (int i = 0; i < RESOURCE_COUNT; ++i) {
			for (int j = 0; j < res_paths_[i].first.size(); ++j) {
				auto& prev = res_paths_[i].first;
				auto& next = res_paths_[i].second;

				int OT = (*node)[prev[j]].row_ != -1 ?
					std::max(0, delay_[prev[j]] - (*node)[prev[j]].v_[0]) : INT_MAX;
				int RT = (*node)[next[j]].row_ != -1 ? 
					std::max(0, delay_[next[j]] - (*node)[next[j]].v_[0]) : 0;

				OTs_and_RTs[i].first.push_back(OT);
				OTs_and_RTs[i].second.push_back(RT);
			}
		}

		for (int i = 0; i < RESOURCE_COUNT; ++i) {
			int min_diff = INT_MAX;
			for (int j = 0; j < res_paths_[i].first.size(); ++j) {
				if ((*node)[res_paths_[i].first[j]].row_ != -1) {
					min_diff = std::min(
						std::max(OTs_and_RTs[i].first[j], 
							OTs_and_RTs[i].second[j]) - OTs_and_RTs[i].second[j],
						min_diff);
				}
			}
			if (min_diff != INT_MAX) { idle_time += vec[i] * min_diff; }
		}

		return heuristicsOne(node, idle_time);
	}

	/* 启发式(3) */
	float heuristicsThree(ptrNode& node) {
		/*
		记录操作资源后置变迁的前置库所及前面的库所个数k
		*/
		vector<int> count(resources_.size(), 0);

		for (int j = 0; j < res_paths_.size(); ++j) {
			for (int& id : res_paths_[j].first) {
				auto pos = findPosition(id);
				std::for_each(paths_[pos.first].begin(),
					paths_[pos.first].begin() + pos.second + 1,
					[&](int& i) {
						if ((*node)[i].row_ != -1) {
							count[j] += (*node)[i].tokens_;
						}
				});
			}
		}

		return heuristicsTwo(node, count);
	}

	/* 启发式(4) */
	float heuristicsFour(ptrNode& node) {
		// ER:资源个数  time:总时间
		float time = 0;

		// 累加各个路径上的工件到达各自目标库所的时间
		for (auto& p : node->state_) {
			// 查询结果：存在返回(路径i, 库所j) 不存在返回(-1, -1)
			std::pair<int, int> res = findPosition(p.row_);
			if (res.first == -1) {
				continue;
			}

			float temp = 0;
			// 累加当前库所还需等待的时间
			temp += std::max(0, delay_[p.row_] - p.v_[std::min(2, (int)p.tokens_) - 1]);
			// 累加后续库所还需等待的时间
			for (int i = res.second + 1; i < paths_[res.first].size(); ++i) {
				temp += delay_[paths_[res.first][i]];
			}
			
			time = std::max(time, temp);
		}
		return time;
	}

	/* 启发式(5) */
	float heuristicsFive(ptrNode& node) {
		// 每个资源的耗费时间
		vector<float> time(RESOURCE_COUNT, 0);

		for (int i = 0; i < RESOURCE_COUNT; ++i) {
			// 计算操作库所的剩余等待时间
			std::for_each(work_by_resource_[i].begin(),
				work_by_resource_[i].end(),
				[&](const int& id) {
					if ((*node)[id].row_ != -1) {
						Place p = (*node)[id];
						float remain = std::accumulate(
							p.v_.begin(),
							p.v_.begin() + std::min((int)p.tokens_, 2),
							0.0,
							[&](const float& num, int16_t& v) {
								return num + std::max(delay_[id] - v, 0);
							});
						time[i] += remain / (*root_)[resources_[i]].tokens_;
					}
				});

			// 计算未来需用到资源的最少消耗代价
			for (auto& p : node->state_) {
				if (p.row_ >= resources_[0] ||
					(i > ROBOT_COUNT - 1 && (p.row_ == 4 || p.row_ == 5))) {
					continue;
				}
				
				float spend_time = 0;
				std::pair<int, int> pos = findPosition(p.row_);
				for (int j = pos.second + 1; j < paths_[pos.first].size(); ++j) {
					if (work_by_resource_[i].count(paths_[pos.first][j])) {
						spend_time += p.tokens_ * delay_[paths_[pos.first][j]];
					}
				}

				time[i] += spend_time / (*root_)[resources_[i]].tokens_;
			}
		}

		return *std::max_element(time.begin(), time.end());
	}

	/* 使能变迁判断 */
	vector<int> enableTrans(ptrNode node) {
		vector<int> ans;
		for (int i = 0; i < num_transition_; ++i) {
			if (*node > Tpre_[i])
				ans.push_back(i);
		}
		return ans;
	}

	/* 目标节点判断 */
	bool isGoalNode(ptrNode curnode) {
		if (*(curnode) > goal_) {
			return 1;
		}
		return 0;
	}

	/* 计算已等待时间v、还需等待最长时间λ */
	int updateVk(ptrNode newnode, ptrNode curnode, int t) {
		int lambda = 0;
		for (int i = 0; i < num_place_; ++i) {
			if (Tpre_[t][i] != 0) {
				// [i]为Node类中的符号重载
				// 意为查看第i个库所是否为空
				// 若为空，返回place->row_ = -1
				// 否则库所i的状态
				if ((*curnode)[i].row_ == -1) {
					printf("something wrong in 110 rows\n");
					exit(-1);
				}
				// 获取输入库所的已等待时间数组
				auto cur_v = (*curnode)[i].v_;
				// v中值最大的指针
				auto v_max = std::max_element(cur_v.begin(), cur_v.end());
				// 还需等待时间
				int diff = delay_[i] - *v_max;
				// 若输入库所激发后还有token 
				if ((*newnode)[i].row_ != -1) {
					// 最大值在v中的索引
					int index = std::distance(cur_v.begin(), v_max);
					// 激发token的已等待时间清零
					(*newnode)[i].v_[index] = 0;
					// 判断v是否从大到小
					bool is_update = std::is_sorted((*newnode)[i].v_.begin(), (*newnode)[i].v_.end(),
						[](int16_t& i1, int16_t& i2) {
							if (i1 == i2) return false;
							return i1 > i2;
						});
					// 若不满足则更新
					if (!is_update) {
						std::sort((*newnode)[i].v_.begin(), (*newnode)[i].v_.end(), std::greater_equal<int16_t>());
					}
				};
				// 还需等待的最大时间
				lambda = std::max(lambda, diff);
			}
		}
		// 更新newnode中的v
		for (int i = 0; i < newnode->state_.size(); ++i) {
			/**
			 * 激发变迁时，Place有两种情况：
			 * 情况一: 静态库所(库所中的token在激发过程中不起作用)
			 *         Tpost_[t][newnode->state_[i].row_] = 0
			 * 情况二：动态库所(库所中的token发生变化)
			 *         Tpost_[t][newnode->state_[i].row_] = 1
			 */
			 // 需更新的token数量
			int num = std::min((int)newnode->state_[i].tokens_, newnode->state_[i].tokens_ - Tpost_[t][newnode->state_[i].row_]);
			for (int j = 0; j < std::min(num, 2); ++j) {
				auto& v = newnode->state_[i].v_;
				v[j] = std::min(delay_[newnode->state_[i].row_], v[j] + lambda);
			}
		}
		return lambda;
	}

	/* 转换为数据集格式进行预测 */
	vector<float> toNetData(const ptrNode node) {
		int j = 0;
		vector<float> ans;
		for (int i = 0; i < num_place_; ++i) {
			if (ignore_m.count(i)) {
				continue;
			}
			if (j < node->state_.size()) {
				if (i < node->state_[j].row_) {
					ans.push_back(0);
				}
				else if (i == node->state_[j].row_) {
					ans.push_back(node->state_[j].tokens_);
					++j;
				}
				else {
					--i; ++j;
				}
			}
			else {
				ans.push_back(0);
			}
		}
		j = 0;
		for (int i = 0; i < num_place_; ++i) {
			if (ignore_v.count(i)) {
				continue;
			}
			if (j < node->state_.size()) {
				if (i < node->state_[j].row_) {
					ans.insert(ans.end(), 2, 0);
				}
				else if (i == node->state_[j].row_) {
					auto v = node->state_[j].v_;
					ans.insert(ans.end(), v.begin(), v.end());
					++j;
				}
				else {
					--i; ++j;
				}
			}
			else {
				ans.insert(ans.end(), 2, 0);
			}
		}
		j = 0;
		for (int i = 0; i < num_place_; ++i) {
			if (ignore_v.count(i)) {
				continue;
			}
			if (j < node->state_.size()) {
				if (i < node->state_[j].row_) {
					ans.push_back(0);
				}
				else if (i == node->state_[j].row_) {
					ans.push_back(delay_[i]);
					++j;
				}
				else {
					--i; ++j;
				}
			}
			else {
				ans.push_back(0);
			}
		}
		return ans;
	}

	/* 变迁激发过程 */
	void fire(ptrNode curnode, int t) {
		auto newnode = *curnode + C[t];
		++total_;
		newnode->discarded_ = false;
		int waiting_time = updateVk(newnode, curnode, t);
		newnode->g_ = curnode->g_ + waiting_time;
		newnode->fathers.emplace_back(std::make_tuple(t, curnode->id_, waiting_time, curnode));
		if (isGoalNode(newnode)) {
			open_list_.push(newnode);
			goal_nodes_.push_back(newnode);
			return;
		}
		switch (SELETE_H) {
			case 1: 
				newnode->h_ = heuristicsOne(newnode);
				break;
			case 2: 
				newnode->h_ = heuristicsTwo(newnode);
				break;
			case 3: 
				newnode->h_ = heuristicsThree(newnode);
				break;
			case 4: 
				newnode->h_ = heuristicsFour(newnode);
				break;
			case 5: 
				newnode->h_ = heuristicsFive(newnode);
				break;
			case 6: 
				//newnode->h_ = nn_->predict(toNetData(newnode));
				break;
			default:
				break;
		}
		auto str = newnode->toString();
		auto pair = isNewNode(newnode);   
		if (pair.second == nullptr) {
			list<ptrNode> temp;
			temp.push_back(newnode);
			entire_list_.emplace(str, std::move(temp));
			open_list_.push(newnode);
			newnode->is_open_ = true;
		}
		else {
			/* 不是新节点，回收 */
			if (!pair.first) {
				pool_.recycling(newnode);
				return;
			}
			/* 是新节点，放入open_list和entire_list */
			else {
				pair.second->emplace_back(newnode);
				open_list_.push(newnode);
				newnode->is_open_ = true;
			}
		}
	}

	/*  minimal G  */
	bool isGless(ptrNode& newnode, ptrNode& oldnode) {
		for (int i = 0; i < newnode->state_.size(); ++i) {
			if (newnode->state_[i].v_ != oldnode->state_[i].v_) {
				return false;
			}
		}
		if (newnode->g_ < oldnode->g_) {
			return false;
		}
		else {
			std::for_each(newnode->fathers.cbegin(),
				newnode->fathers.cend(),
				[&](auto t) {
					oldnode->fathers.emplace_back(t);
				});
		}
		return true;
	}

	/* 弱时间轴判断(是否为旧节点) */
	bool isOld(ptrNode& newnode, ptrNode& oldnode) {
		/* g值差 */
		int diff = newnode->g_ - oldnode->g_;
		// 将v和v'转移到同一时间点比较
		for (int i = 0; i < newnode->state_.size(); ++i) {
			if (newnode->state_[i].v_[0] > oldnode->state_[i].v_[0] + diff ||
				newnode->state_[i].v_[1] > oldnode->state_[i].v_[1] + diff) {
				return false;
			}
		}
		return true;
	}

	/* 新旧节点判断 */
	std::pair<bool, list<ptrNode>*> isNewNode(ptrNode newnode) {
		auto str = newnode->toString();
		if (entire_list_.count(str) <= 0) {
			return std::make_pair(1, nullptr);
		}
		auto it = entire_list_.find(str);

		/* 与链表上的每个节点依次进行比较 */
		for (auto itor = it->second.begin(); itor != it->second.end();) {
			auto oldnode = *itor;
			/* 判断新拓展出来节点的新旧性 */
			if (isOld(newnode, oldnode)) {
				return std::make_pair(0, &(it->second));
			}
			/* 判断是否删除旧节点 */
			if (isOld(oldnode, newnode)) {
				// 只回收不在open表中的节点
				if (!oldnode->is_open_) {
					pool_.recycling(oldnode);
				}
				else {
					oldnode->discarded_ = true;
				}
				// 从close表中删除
				itor = it->second.erase(itor);
				continue;
			}
			++itor;
		}
		return std::make_pair(1, &(it->second));
	}

	/* Astar搜索 */
	void AstarSearch() {
		std::cout << "\nBegin forward tree -> ";
		clock_t start = clock();
		open_list_.push(root_);

		while (!open_list_.empty()) {
			auto curnode = open_list_.top();
			curnode->is_open_ = false;
			open_list_.pop();

			if (curnode->discarded_) {
				continue;
			}

			if (isGoalNode(curnode)) {
				g_min_ = curnode->g_;
				break;
			}

			auto enables = enableTrans(curnode);
			if (enables.empty()) {
				curnode->is_deadlock_ = true;
				continue;
			}
			for (auto t : enables) {
				fire(curnode, t);
			}
		}
		clock_t end = clock();
		auto programTimes = end - start;
		std::cout << "Forward tree finish(" << programTimes << "ms)\t";
	}

	/* 集束搜索 */
	void BeamSearch(int k) {
		/* K表 */
		std::priority_queue<ptrNode, vector<ptrNode>, AStar> k_list;

		clock_t start = clock();
		/* 插入根节点 */
		open_list_.push(root_);
		while (!open_list_.empty()) {
			/* open表前K个节点 */
			for (; k_list.size() < k && !open_list_.empty(); ) {
				auto top_node = open_list_.top();
				k_list.push(top_node);
				top_node->is_open_ = false;
				open_list_.pop();
			}
			/* 置换 */
			open_list_ = std::move(k_list);

			/* 取节点扩展 */
			auto curnode = open_list_.top();
			open_list_.pop();
			/* 旧节点 */
			if (curnode->discarded_) {
				continue;
			}
			/* 目标节点 */
			if (isGoalNode(curnode)) {
				g_min_ = curnode->g_;
				break;
			}
			/* 激发 */
			auto enables = enableTrans(curnode);
			if (enables.empty()) {
				curnode->is_deadlock_ = true;
				continue;
			}
			for (auto t : enables) {
				fire(curnode, t);
			}
		}
		clock_t end = clock();
		auto programTimes = end - start;
		std::cout << "Forward tree finish（" << programTimes << "ms）\t";
	}

	/* 结果信息 */
	static void infoDisplay(void* petri) {
		PetriNet* net = static_cast<PetriNet*>(petri);

		std::cout << "\nExpand Nodes: " << net->total_ << "\n";
		std::cout << "Makespan: " << net->g_min_ << std::endl;
	}

	~PetriNet() {}
};