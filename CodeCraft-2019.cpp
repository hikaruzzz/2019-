#include<iostream>
#include<stdio.h>
#include<algorithm>
#include<list>
#include<fstream>
#include<deque>
#include<vector>
#include<string>
#include<cstring>
#include<map>
#include<time.h>
using namespace std;

const int Max_V = 180;  //最大节点数 + 1（cross数）【注：整个邻接矩阵以1开始，0是特殊值】
const int Inf_Value = 9999999;  //无穷值
const int Max_Car = 62000;  //车辆总数
const int Max_Plan_Route = 180;  //每辆车 的最大计划路径 节点数
const int Max_Speed = 16;  //车辆最大速度
const int Max_K = Max_Speed;  //分group时，最大K，如按速度分组，K=8
const int Max_K_CarDist = 180;  //最大车程（from -to）
const int Max_Plantime = 10000;
//邻接矩阵 
int road_length[Max_V][Max_V];   //道路长
int road_index[Max_V][Max_V];  //道路id
float road_capacity[Max_V][Max_V];  //道路容量（初始为4车道，容量为4）
int road_speed[Max_V][Max_V];  //道路限速度
int road_time_cost[Max_V][Max_V];  //道路花费
int road_capacity_backup[Max_V][Max_V];   //道路capacity
int road_capacity_true[Max_V][Max_V];  //道路的真实 容量
//dict of cross id
map<int, int> crossid_dict;

//BEGIN of WJK
enum txtType { CAR, CROSS, ROAD };
struct Cross {
	int id, NRoad_id, ERoad_id, SRoad_id, WRoad_id;
};
struct Road {
	int id, length, speed, channel, from, to, isDuplex;
};
struct Car
{
	int id;
	int from;
	int to;
	int speed;
	int plan_time;
	//Car(int id, int speed, int from, int to, int plan_time) :id(id), speed(speed), from(from), to(to), plan_time(plan_time) {};

};

void strtrim(string &s) {
	int index = 0;
	if (!s.empty())
	{
		while ((index = s.find(' ', index)) != string::npos)
		{
			s.erase(index, 1);
		}
	}

}

void strSplit(const string& s, vector<string>& v, const string& c)
{
	string::size_type pos1, pos2;
	pos2 = s.find(c);
	pos1 = 0;
	while (string::npos != pos2)
	{
		v.push_back(s.substr(pos1, pos2 - pos1));

		pos1 = pos2 + c.size();
		pos2 = s.find(c, pos1);
	}
	if (pos1 != s.length())
		v.push_back(s.substr(pos1));
}

Cross str2Cross(vector<int> &params) {

	Cross c = { params[0],params[1] ,params[2] ,params[3] ,params[4] };
	return c;
}

int fakeID2Real(int fakeid) {
	for (auto _find = crossid_dict.begin(); _find != crossid_dict.end(); _find++) {
		if (_find->second == fakeid)
			return _find->first;
	}


	return -2;
}
Road str2Road(vector<int> &params) {
	int from = fakeID2Real(params[4]), to = fakeID2Real(params[5]);
	//cout << from << "->>" << to << endl;
	Road r = { params[0],params[1] ,params[2] ,params[3] ,from,to,params[6] };
	return r;
}

Car str2Car(vector<int> &params) {

	Car c = { params[0],fakeID2Real(params[1]) ,fakeID2Real(params[2]) ,params[3] ,params[4] };
	return c;
}

void ReadData(const string &filepath, txtType type, list<Car> &carList, list<Road> &roadList, list<Cross> &crossList) {


	ifstream fin(filepath);
	const int LINE_LENGTH = 100;
	char str[LINE_LENGTH];
	while (fin.getline(str, LINE_LENGTH))
	{


		if (str[0] == '#')
			continue;
		string line = str;
		strtrim(line);
		line = line.substr(1, line.length() - 2);

		vector<string> s;
		strSplit(line, s, ",");
		vector<int> params(10);
		params.clear();
		for (int i = 0; i < s.size(); i++) {
			int a = atoi(s[i].data());
			params.push_back(a);
		}

		switch (type) {
		case CROSS: {
			Cross cross = str2Cross(params);
			crossList.push_back(cross);
			break;
		}
		case ROAD: {
			Road road = str2Road(params);
			roadList.push_back(road);
			break; }
		case CAR: {
			Car car = str2Car(params);
			carList.push_back(car);
		}
		}
	}
}

int SPFA(int s, int to, int road_time_cost[][Max_V], int *pre_route);
int original_dist_cost[Max_V][Max_V];
void OriginalCost(int road_length[][Max_V], float road_capacity[][Max_V]) {
	int simpleDistance = Max_V;
	memset(original_dist_cost[0], 0, sizeof(int)*Max_V);
	for (int i = 0; i < Max_V; i++) {
		for (int j = 0; j < Max_V; j++) {
			if (j == 0)
				original_dist_cost[i][j] = Inf_Value;
			else
			{
				int pre_route[Max_Plan_Route] = { };  //存放路径，默认为0
				simpleDistance = SPFA(i, j, road_length, pre_route);
				if (road_capacity[i][j] < 2) {
					//加上道路容量
					//simpleDistance += 1;
				}

				original_dist_cost[i][j] = simpleDistance;
			}
		}
	}

	/*for (int i = 0; i < Max_V; i++) {
		for (int j = 0; j < Max_V; j++) {
			cout << original_dist_cost[i][j] << ' ';
		}
		cout << endl;
	}
*/
}


//End of WJK
/* ===================================*/
void InitRoad(int road_length[][Max_V], int road_index[][Max_V], float road_capacity[][Max_V], int road_speed[][Max_V],
	list<Car> &carList, int road_capacity_backup[][Max_V], string &carPath, string &roadPath, string &crossPath)
{
	//	 road_length[Max_V][Max_V];
	//	 road_index[Max_V][Max_V];
	//	 road_capacity[Max_V][Max_V];  //道路容量（初始为4车道，容量为4）,要求：[i][i]的值为-1
	//	 road_speed[Max_V][Max_V];  //道路限速度，若capacity=0时，改值为-1
	for (int i = 0; i < Max_V; i++)
	{
		memset(road_index[i], -1, sizeof(int)*Max_V);
		memset(road_capacity[i], 0, sizeof(int)*Max_V);
		memset(road_capacity_backup[i], 0, sizeof(int)*Max_V);
		memset(road_speed[i], 0, sizeof(int)*Max_V);
		for (int j = 0; j < Max_V; j++)
		{
			road_length[i][j] = Inf_Value;
		}
	}

	list<Cross> crossList;
	list<Road> roadList;
	ReadData(crossPath, CROSS, carList, roadList, crossList);

	int _iter_count = 1;
	for (auto iter = crossList.begin(); iter != crossList.end(); iter++) {
		crossid_dict.insert(pair<int, int>(_iter_count, (*iter).id));
		_iter_count++;
	}

	ReadData(carPath, CAR, carList, roadList, crossList);
	ReadData(roadPath, ROAD, carList, roadList, crossList);


	int roadNum = roadList.size();
	for (list<Road>::iterator iter = roadList.begin(); iter != roadList.end(); iter++) {
		int from = (*iter).from, to = (*iter).to;

		road_index[from][to] = (*iter).id;
		road_length[from][to] = (*iter).length;
		road_capacity[from][to] = (*iter).channel;
		road_capacity_backup[from][to] = (*iter).channel;
		road_speed[from][to] = (*iter).speed;
		if ((*iter).isDuplex == 1) {
			road_index[to][from] = (*iter).id;
			road_length[to][from] = (*iter).length;
			road_capacity[to][from] = (*iter).channel;
			road_capacity_backup[to][from] = (*iter).channel;
			road_speed[to][from] = (*iter).speed;
			roadNum++;
		}
	}
}

int g_i = 0;
void GetPathSPFA(int to, int *pre_route, int *pre_path)
{
	if (pre_path[to] != 0)
		GetPathSPFA(pre_path[to], pre_route, pre_path);
	pre_route[g_i] = to;
	g_i++;
}

int SPFA(int s, int to, int road_time_cost[][Max_V], int *pre_route)
/*
paras: s:start
*/
{

	int visited[Max_V] = {};
	int pre_path[Max_V] = {};
	int dist[Max_V] = { Inf_Value };
	int i;
	int	temp;
	deque<int> q;
	for (int i = 0; i < Max_V; i++)
		dist[i] = Inf_Value;
	dist[s] = 0;
	visited[s] = 1;

	q.push_back(s);
	while (!q.empty())
	{
		temp = q.front();  //！队列前
		q.pop_front();
		visited[temp] = 0;
		for (i = 0; i < Max_V; i++)
		{
			if (dist[temp] + road_time_cost[temp][i] < dist[i])
			{
				dist[i] = dist[temp] + road_time_cost[temp][i];
				pre_path[i] = temp;

				if (!visited[i])
				{
					q.push_back(i);
					visited[i] = 1;
				}
			}
		}
	}
	g_i = 0;
	GetPathSPFA(to, pre_route, pre_path);
	return g_i;
}

int gi = 0;   //【此处bug，无法重置，g++不断大，内存异常】
void GetPredictPath(int path[][Max_V], int from, int to, int *pre_route)
{

	//利用递归方法，从前驱矩阵中获得路径
	if (from == to || to == Inf_Value)
	{
		pre_route[gi] = from;
		gi++;
	}
	else
	{
		GetPredictPath(path, from, path[from][to], pre_route);
		pre_route[gi] = to;
		gi++;
	}
}

void Floyd(int road_time_cost[][Max_V], int from, int to, int *pre_route)
/*
【该算法，无法求带回环图，放弃】
Floyd算法
paras:
return: null
*/
{
	//test print

	int tmp;
	int dist[Max_V][Max_V];  //最短路径 权重矩阵
	int path[Max_V][Max_V];  //前驱节点矩阵

	//初始化
	for (int i = 0; i < Max_V; i++)
	{
		for (int j = 0; j < Max_V; j++)
		{
			dist[i][j] = road_time_cost[i][j];
			path[i][j] = j;
		}
	}

	//算法核心
	//【注：每次 调用后，road_time_cost邻接矩阵都会改变，可以print出来看看】
	for (int k = 0; k < Max_V; k++)
	{
		for (int i = 0; i < Max_V; i++)
		{
			for (int j = 0; j < Max_V; j++)
			{
				//计算前驱节点
				if (k == 0)
				{
					if ((i == j) || (road_time_cost[i][j] == Inf_Value))
						path[i][j] = Inf_Value;
					else
						path[i][j] = i;
				}
				else
				{
					if (dist[i][j] > dist[i][k] + dist[k][j])
						path[i][j] = path[k][j];
				}
				//若 经过 k顶点比原路更短，则更新dist
				int tmp = (dist[i][k] == Inf_Value || dist[k][j] == Inf_Value) ? Inf_Value : (dist[i][k] + dist[k][j]);
				if (dist[i][j] > tmp)
				{
					dist[i][j] = tmp;
				}
			}
		}
	}

	//根据 前驱矩阵，输入from，to节点，获得最短路径
	gi = 0;
	if (dist[from][to] < Inf_Value)
		GetPredictPath(path, from, to, pre_route);
	//else
		//cout << " null pre_path\n";
}


void RenewRoadTimeCost(int road_time_cost[][Max_V], float road_capacity[][Max_V], int road_length[][Max_V], int road_speed[][Max_V], int this_car_speed, int start_time_distance)
/*
更新 road_time_cost邻接矩阵
paras: **road_time_cost,**road_capacity, **road_length,**road_speed ,this_car_speed当前车的速度，
		start_time_distance 这一组的第一辆车发车时间 - 该辆车发车时间
return: null 【指针改值】
*/
{
	for (int i = 0; i < Max_V; i++)
	{
		for (int j = 0; j < Max_V; j++)
		{
			if (i != 0 && j != 0)  //不干扰虚拟起点0
			{
				if (road_speed[i][j] == 0)
				{
					road_time_cost[i][j] = Inf_Value;
				}
				else
				{
					road_time_cost[i][j] = road_length[i][j] / min(this_car_speed, road_speed[i][j]); //cost = S/Vmin
				}


				//判断：如果该道路的capacity = 0，则不允许通过，cost = Inf_Value
				if (road_capacity[i][j] <= 0 || i == j)
				{
					road_time_cost[i][j] = Inf_Value;
				}
			}
			else
			{
				//设定虚拟起点：time_cost邻接矩阵 的 0行，0列，是指虚拟起始点
				//如：road_time_cost[0][4] 即为 虚拟起始点到 节点4 的花费时间（为该辆车发车time - 第一辆车发车time）
				road_time_cost[0][j] = start_time_distance;
			}
		}
		road_time_cost[i][0] = Inf_Value; //不需要考虑 出发点去虚拟起始点的情况
	}
}

void BlockCapByCapOccupy(float road_capacity[][Max_V], int road_capacity_true[][Max_V], float zero_cap_threshold)
/*
按照 capacity占用情况，给定此路是否可通行 road_capacity[i][j] = 0;
*/
{
	for (int i = 1; i < Max_V; i++)
	{
		for (int j = 1; j < Max_V; j++)
		{
			if (road_capacity[i][j] <= zero_cap_threshold * road_capacity_true[i][j])  //当capacity小于 【真实容量 * zero_cap_threshold】，禁止通行
			{
				road_capacity[i][j] = 0;
			}
		}
	}
}

void UpdateRoadCapacity(float road_capacity[][Max_V], int *pre_route)
/*
更新 road_capacity
paras: road_capacity[][],  pre_route[] 要求下标从0开始，pre_route[0] 为from节点
*/
{
	//Update  road_capacity
	for (int i = 0; i <= sizeof(pre_route); i++)
	{
		road_capacity[pre_route[i]][pre_route[i + 1]] -= (1 - i / sizeof(pre_route));
		//road_capacity[pre_route[i]][pre_route[i + 1]] -= 1;
	}
}

void TurboRoadCapacity(float road_capacity[][Max_V], int  road_capacity_true[][Max_V], int road_capacity_backup[][Max_V], int road_length[][Max_V])
/*
道路Capacity 扩展 为真实容量
paras: turbo_capacity_trim 微调值
*/
{
	for (int i = 0; i < Max_V; i++)
	{
		for (int j = 0; j < Max_V; j++)
		{
			if (road_capacity[i][j] != 0)
			{
				road_capacity[i][j] = road_capacity_true[i][j] = road_capacity_backup[i][j] * road_length[i][j];
			}
		}
	}
}

void AddRoadCapCapacity(float road_capacity[][Max_V], int road_capacity_true[][Max_V], float add_cap_rate)
/*
修改 道路占有率 ，增加【 真实容量 * add_cap_rate 】
*/
{

	for (int i = 1; i < Max_V; i++)
	{
		for (int j = 1; j < Max_V; j++)
		{
			road_capacity[i][j] = road_capacity[i][j] + road_capacity_true[i][j] * add_cap_rate;
			//road_capacity[i][j] = road_capacity_true[i][j] * add_cap_rate;

			if (road_capacity[i][j] >= road_capacity_true[i][j])
				road_capacity[i][j] = road_capacity_true[i][j];
		}
	}
}

void ResetRoadCapacity(float road_capacity[][Max_V], int road_capacity_backup[][Max_V])
/*
重置  road_capacity
paras: road_capacity[][],  pre_route[] 要求下标从0开始，pre_route[0] 为from节点
*/
{
	//reset  road_capacity
	for (int i = 0; i < Max_V; i++)
	{
		for (int j = 0; j < Max_V; j++)
		{
			road_capacity[i][j] = road_capacity_backup[i][j];
			//cout << road_capacity[i][j] << "  ";
		}
		//cout << endl;
	}
}

void CarGroupRangeOfSpeed(Car *cars, deque<Car> *car_group, Car car_null)
/*
车辆分组 按速度
*/
{
	for (int k = Max_K; k > 0; k--)  //k=0: 速度为k 的车
	{
		//cout << "cars: " << cars[k].id << "   ";
		for (int i = 0; i < Max_Car; i++)
		{
			if (cars[i].speed == k)
			{
				car_group[k].push_back(cars[i]);  //从大到小出发
			}
		}
		car_group[k].push_back(car_null);  //每个group的最后push一个null车【当检测到.front().id为-1时，则说明这去k节点一组没有车】
	}
}

void CarGroupRangeOfPlanTime(Car *cars, deque<Car> *car_group, Car car_null)
/*
按时间 排组 */
{
	for (int i = 0; i < Max_Car; i++)
	{
		if (cars[i].id != -1)
			car_group[cars[i].plan_time].push_back(cars[i]);
	}
	for (int j = 0; j <= Max_Plantime; j++)
	{
		car_group[j].push_back(car_null);
	}

}

void CarGroupRangeOfDist(Car *cars, deque<Car> *car_group, Car car_null, float road_capacity[][Max_V], int road_length[][Max_V], int road_speed[][Max_V])
/*
车辆分组 按距离
*/
{
	int _road_time_cost[Max_V][Max_V];
	int pre_route[Max_Plan_Route] = {};
	//k=0: 速度为k 的车
	int this_car_speed;
	int start_time_distance = 0;
	for (int i = 0; i < Max_Car; i++)
	{
		if (cars[i].id != -1)
		{
			this_car_speed = cars[i].speed;
			RenewRoadTimeCost(_road_time_cost, road_capacity, road_length, road_speed, this_car_speed, start_time_distance);
			int cur_dis = original_dist_cost[cars[i].from][cars[i].to]; // SPFA(cars[i].from, cars[i].to, _road_time_cost, pre_route);
			car_group[cur_dis].push_back(cars[i]);  //距离长的先安排

		}
	}
	for (int k = 0; k < Max_K_CarDist + 1; k++)
	{
		car_group[k].push_back(car_null);  //每个group的最后push一个null车【当检测到.front().id为-1时，则说明这去k节点一组没有车】
	}
}

bool MyCompareSpeed(Car c1, Car c2)
/*
自定义list.sort排序方法【按 速度】
注意car_group[]最后一个是 -1 车 ,后面读取时是.back(),所以为反转排序
*/
{
	return c1.speed < c2.speed;  //使得最 前面 一个是 -1，读取时从后往前读到-1
}

bool MyCompareDist(Car c1, Car c2)
/*
自定义list.sort排序方法【按】
注意car_group[]最后一个是 -1 车 ,后面读取时是.back(),所以为反转排序
*/
{
	if (c1.id == -1)
		return true;
	if (c2.id == -1)
		return false;
	return original_dist_cost[c1.from][c1.to] > original_dist_cost[c2.from][c2.to];
	//从大到小排序，读取后是 先跑小的再跑大
}

Car SetNullCar()
{
	//生成  一个null车
	Car cars;
	cars.id = -1;
	cars.plan_time = -1;
	cars.speed = -1;
	cars.from = -1;
	cars.to = -1;

	return cars;
}

void SetCars(Car *cars, list<Car> &carlist)
{
	int i = 0;
	Car null_car = SetNullCar();
	for (int j = 0; j < Max_Car; j++)
	{
		cars[j] = null_car;
	}

	while (!carlist.empty()) {
		cars[i] = carlist.front();
		carlist.pop_front();
		i++;
	}
}

//车辆集
list<Car> carList; //用了list
//deque<Car> car_group[Max_K_CarDist + 1];  //按到达终点划分的 车组 ,类型为Car,下标为终点节点【从1节点开始】
deque<Car> car_group[Max_Plantime + 1];  //按到达终点划分的 车组 ,类型为Car,下标为终点节点【从1节点开始】

Car cars[Max_Car]; //汽车 存储数组

int main(int argc, char *argv[])
{
	//int start_time = clock();
	string carPath(argv[1]);
	string roadPath(argv[2]);
	string crossPath(argv[3]);
	string answerPath(argv[4]);

	//running info 
	std::cout << "Begin" << std::endl;

	if (argc < 5) {
		std::cout << "please input args: carPath, roadPath, crossPath, answerPath" << std::endl;
		exit(1);
	}


	std::cout << "carPath is " << carPath << std::endl;
	std::cout << "roadPath is " << roadPath << std::endl;
	std::cout << "crossPath is " << crossPath << std::endl;
	std::cout << "answerPath is " << answerPath << std::endl;

	//crossid_dict.clear();


	//车辆集
	list<Car> carList; //用了list

	int this_car_speed = 1;  //要规划的车 速  [为0,error]
	int start_time_distance = 0;  //发车 时间距离


	Car car_null = SetNullCar(); //生成 null 车
	//规划集
	deque<int> plan_map_point[Max_Car];   //最短路径 节点表【存放int数组】  【若规划路线过长，可能不止Max_V的大小】
	list<int> plan_map_road[Max_Car];   //最短路径  路表

	//读取cross.txt,road.txt，初始化邻接表
	InitRoad(road_length, road_index, road_capacity, road_speed, carList, road_capacity_backup, carPath, roadPath, crossPath);

	//if(road_index[27][35] == 5050&&road_length[10][11] == 15 &&)


	//读car.txt,转成car结构体数组car[]
	SetCars(cars, carList);
	OriginalCost(road_length, road_capacity);

	//汽车分组car_group[n]（按速度  分组）
	//【新想法？：按距离分组？按出发时间分组，同一时刻出发为一组？】
	//CarGroupRangeOfSpeed(cars, car_group, car_null);

	int count_car = 0;  //输出数组 车 计数器
	int from;  //单辆车 from，to
	int to;  //to
/*==========================================================================================================================*/
	float zero_cap_threshold = 0.;  //【设为0，下面add_cap和ADD_CAP动就行】【看作 保留空位，与add_cap_rate差值越大越快】当capacity 小于 【真实容量 * zero_cap_threshold】，禁止通行当capacity
	float add_cap_rate = 0.01;  //【看作每gap_time_add_ADD_CAP， 车到站数】【一段时间后，基本都空】 每隔一个gap_time_add_ADD_CAP,增加capacity 一定比例【真实容量 * add_cap_rate】 【减少占用情况】
	float wjk_value = 1.8;  //往小调，分的路长度减小，gap次数变多，所以 cout_gaptime_B 适当增加
	//int cout_gaptime_B = 60;  //gap time add 为 0的次数 【所有组总次数】
	//int gap_time_add = 1;  //发车间隔 步长  【可以 微调】【每次遍历一次组，gap_time += gap_time_add
	int gap_time_add_ADD_CAP = 50;  //【绑定add_cap_rate用，调小，总出发时减小】使用一次 add_road_cap 函数 增加capacity 的时间间隔【每个时间片，增加gap_time，减少一定 拥堵程度】
/*==========================================================================================================================*/
	int gap_time = 0;  //发车间隔【plan_time+gap_time = 实际出发时间】
	Car car_temp;  //当前车辆 指示器


	//capacity 打鸡血【扩充 road_capacity 为真实容量】
	TurboRoadCapacity(road_capacity, road_capacity_true, road_capacity_backup, road_length);
	CarGroupRangeOfPlanTime(cars, car_group, car_null);
	//CarGroupRangeOfDist(cars, car_group, car_null, road_capacity, road_length, road_speed);

	int T = 0;
	int cout_gaptime = 0;
	//-----------------循环 寻路  -----
	//for (int k = Max_Speed; k >= 1; k--) 
	//for (int k = 1; k <= Max_K_CarDist; k++)  //从小到大【距离】 分路
	for (int k = 1; k <= Max_Plantime; k++)
	{
		//T++;
		//【按 back 顺序读取】
		//car_group[k].sort(MyCompareDist);  //排序，按距离 【【试试扔到while里面，强制让短途车先跑

		//sort(car_group[k].begin(), car_group[k].end(), MyCompareSpeed);
		sort(car_group[k].begin(), car_group[k].end(), MyCompareDist);

		int first_start_time = car_group[k].back().plan_time;  //第 k 组的第一辆车的 出发时间作为 虚拟源点时间

		//每个时间片，增加gap_time，减少一定 拥堵程度
		//【通过修改reduce_cap_occupy_rate值达到最优 间隔】
		//gap_time += gap_time_add_ADD_CAP;  
		//含义：每经过一个时间片 k+1gap_time_add_ADD_CAP，增加 road_容量 【road_capacity_true * add_cap_rate】
		AddRoadCapCapacity(road_capacity, road_capacity_true, add_cap_rate);  //减少 占有率 【增加capacity】

		while (car_group[k].back().id != -1)  //当car_group最前面的nullcar出现在back，说明此组已空
		{
			if (car_group[k].back().id == 0) cout << " !!!!!!error :  back.id = 0   !!!!!!!!!!!";
			int cout_turbo = 0;
			int count_car_temp = car_group[k].size() - 1;  //组内，从后往前 遍历
			int print_car_group = car_group[k].size();

			int car_group_k_size = car_group[k].size();
			for (int loop_num = 0; loop_num <= car_group_k_size; loop_num++)
				//遍历 同组所有车，分配路
			{
				if (car_group[k].back().id == -1)
				{
					cout << " loop_num of car_group[ " << k << " ] loop_num: " << loop_num << " of size: " << print_car_group << "\n";
					break;
				}

				//if (loop_num % 1000 == 0)
				//cout << " loop_num of car_group[ " << k << " ] loop_num: " << loop_num << " of size: " << print_car_group << "\n";

				car_temp = car_group[k][count_car_temp];  //当前车 指示
				count_car_temp--;

				//if (car_temp.id == -1)
				//	continue;  //如果检测到 车（id=-1)则跳过 单车 配路

				//车辆属性 赋值
				this_car_speed = car_temp.speed;
				start_time_distance = car_temp.plan_time - first_start_time;  //距离 虚拟源点 的时长
				from = car_temp.from;
				to = car_temp.to;

				//更新road_time_cost 邻接矩阵	
				RenewRoadTimeCost(road_time_cost, road_capacity, road_length, road_speed, this_car_speed, start_time_distance);

				int pre_route[Max_Plan_Route] = { };  //存放路径，默认为0
				//寻找最短路径,得到 该车规划路 pre_path
				//Floyd(road_time_cost, from, to, pre_route);  //算最短路径，存入pre_route数组【单辆车】
				int cur_dis = SPFA(from, to, road_time_cost, pre_route);


				//判断是否 存在 最短路径
				if (pre_route[1] != 0 && ((float)cur_dis / original_dist_cost[from][to] <= wjk_value))
				{
					//存入 输出list
					plan_map_point[count_car].push_back(car_temp.id);
					plan_map_point[count_car].push_back(car_temp.plan_time);
					for (int i = 0; pre_route[i] != 0; i++)
					{
						plan_map_point[count_car].push_back(pre_route[i]);
					}
					count_car++;

					//删除 当前指示的车
					for (deque<Car>::iterator del_it = car_group[k].begin(); del_it != car_group[k].end(); )
					{  //用迭代器 随机 删除
						if ((*del_it).id != car_temp.id || (*del_it).id == -1)
						{
							del_it++;
						}
						else
						{
							//cout << "delete car:" << (*del_it).id << endl;
							del_it = car_group[k].erase(del_it);
						}
					}
					//更新road_capacity  , road_capacity_occupy 邻接矩阵
					UpdateRoadCapacity(road_capacity, pre_route);

					//把过度拥挤的道路 封闭不通 capacity[i][j] = 0
					BlockCapByCapOccupy(road_capacity, road_capacity_true, zero_cap_threshold);
				}
				else  //该车 无最短路径
				{
					//把该车 push到 + time_gap的组
					car_temp.plan_time += gap_time_add_ADD_CAP;
					car_group[k + gap_time_add_ADD_CAP].push_back(car_temp);
					//删除 当前指示的车
					for (deque<Car>::iterator del_it = car_group[k].begin(); del_it != car_group[k].end(); )
					{  //用迭代器 随机 删除
						if ((*del_it).id != car_temp.id || (*del_it).id == -1)
						{
							del_it++;
						}
						else
						{
							//cout << "delete car:" << (*del_it).id << endl;
							del_it = car_group[k].erase(del_it);
						}
					}
				}
			}
		}
	}

	//输出 规划表 answer.txt
	//(车辆id，实际出发时间，行驶路线序列)【空格隔开】如：(1001, 1， 501, 502, 503, 516, 506, 505, 518, 508, 509, 524)
	ofstream mycout(answerPath);

	for (int i = 0; i < count_car; i++)
	{

		mycout << "(";
		mycout << plan_map_point[i][0] << ",";
		mycout << plan_map_point[i][1];
		for (int j = 2; j < plan_map_point[i].size() - 1; j++)
		{
			mycout << "," << road_index[plan_map_point[i][j]][plan_map_point[i][j + 1]];
		}
		mycout << ")";
		mycout << "\n";

	}
	mycout.close();
	//cout << " used time: " << clock() - start_time << endl;
	cout << "Finish.";

	//getchar();
}
