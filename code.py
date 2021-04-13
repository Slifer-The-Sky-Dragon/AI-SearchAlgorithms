import heapq
import math
import time

NOPAR = -1

ROW_MOVES = [1 , -1 , 0 , 0]
COL_MOVES = [0 , 0 , 1 , -1]

table = []
balls_start = {}
balls_end = {}

names = ['BFS' , 'IDS' , 'First A*' , 'Second A*' , 'weighted A*(alpha = 2)' , 'weighted A*(alpha = 5)']
execution_times = []

class Point:
    def __init__(self , x , y):
        self.x = x
        self.y = y
    def __str__(self):
        return 'Position : (' + str(self.x) + ' , ' + str(self.y) + ') '
    def __eq__(self , other):
        return (self.x == other.x and self.y == other.y)

class State:
    def __init__(self , pos , cor_balls , bag_balls):
        self.pos = pos
        self.cor_balls = cor_balls
        self.bag_balls = bag_balls
        self.par = None
    def __str__(self):
        return str(self.pos) + ' - Correct balls = ' + str(self.cor_balls) + ' - Bag balls = ' + str(self.bag_balls)
    def __eq__(self, other):
        if other == None:
            return False
        return (self.pos == other.pos and self.cor_balls == other.cor_balls and self.bag_balls == other.bag_balls)
    def __lt__(self , other):
        if other == None:
            return False
        return (self.pos.x > other.pos.x)
    def __hash__(self):
        return hash( (self.pos.x , self.pos.y , self.cor_balls , tuple(self.bag_balls)) )
                
def find_move(pos , par_pos):
	if pos == par_pos:
		return 'Pick'
	elif pos.x == par_pos.x + 1:
		return 'Down'
	elif pos.x == par_pos.x - 1:
		return 'Up'
	elif pos.y == par_pos.y + 1:
		return 'Right'
	elif pos.y == par_pos.y - 1:
		return 'Left'
	else:
		return ''

def print_path(state , distinct_states , all_states , enable):
	if enable == False:
		return
	length = 0
	cur_move = 'End'
	print("********printing' path:********")
	while state.par != None:
		length += 1
		print(state , end = ' - ')
		print('move = ' + cur_move)
		cur_move = find_move(state.pos , state.par.pos)
		state = state.par
	print(state , end = ' - ')
	print('move = ' + cur_move)

	print('length of path = ' + str(length))
	print('Number of distinct states = ' + str(distinct_states))
	print('Number of all states = ' + str(all_states))
	print('-------------------------------')

def is_valid_pos(pos , n , m , table):
    if pos.x >= 0 and pos.x < n and pos.y >= 0 and pos.y < m and table[pos.x][pos.y] == '-':
        return True
    return False

def bfs(n , m , start_state , end_state , c , k , balls_start , balls_end , table , start_time , print_enable):
    frontier_set = [start_state]
    explored_set = set([])
    mark = {}
    all_states = 0

    mark[start_state] = True
    while True:
        cur_state = frontier_set.pop(0)
        all_states += 1
        explored_set.add(cur_state)

        if cur_state == end_state:
            end_time = time.time()
            execution_times.append(end_time - start_time)
            if print_enable == True:
            	print('BFS Algorithm path')
            print_path(cur_state , len(explored_set) , all_states , print_enable)
            break

        bag_cnt = 0
        for i in range(k):
            if cur_state.bag_balls[i] == 1:
                bag_cnt += 1

        cur_pos = cur_state.pos

        #pick up
        if ((cur_pos.x , cur_pos.y) in balls_start.keys()) and bag_cnt + 1 <= c:
            ball_id = balls_start[(cur_pos.x , cur_pos.y)]
            if cur_state.bag_balls[ball_id] == 0:
                new_state_bag_balls = cur_state.bag_balls.copy()
                new_state_bag_balls[ball_id] = 1

                new_state = State(cur_state.pos , cur_state.cor_balls , new_state_bag_balls)

                if new_state not in mark.keys():
                    frontier_set.append(new_state)
                    new_state.par = cur_state
                    mark[new_state] = True

        for i in range(4):
            new_x = cur_pos.x + ROW_MOVES[i]
            new_y = cur_pos.y + COL_MOVES[i]
            new_pos = Point(new_x , new_y)

            if is_valid_pos(new_pos , n , m , table) == True:
                new_state_bag_balls = cur_state.bag_balls.copy()
                new_state_cor_balls = cur_state.cor_balls

                if (new_pos.x , new_pos.y) in balls_end.keys():
                    ball_id = balls_end[(new_pos.x , new_pos.y)]
                    if new_state_bag_balls[ball_id] == 1:
                        new_state_bag_balls[ball_id] = 2
                        new_state_cor_balls += 1
                
                new_state = State(new_pos , new_state_cor_balls , new_state_bag_balls)
                if new_state not in mark.keys():
                    frontier_set.append(new_state)
                    new_state.par = cur_state
                    mark[new_state] = True


def dfs(n , m , start_state , end_state , c , k , max_depth , start_time , 
					all_states , explored_set_in , print_enable):
    frontier_set = [(start_state , 0)]
    explored_set = set([])
    mark = {}
    depth = {}

    mark[start_state] = True
    depth[start_state] = 0

    answer_state = None
    while len(frontier_set) != 0:
        cur_state_t = frontier_set.pop()
        cur_state = cur_state_t[0]
        cur_depth = cur_state_t[1]

        all_states += 1
        explored_set.add(cur_state)
        if cur_state == end_state:
            if cur_depth == max_depth:
                end_time = time.time()
                execution_times.append(end_time - start_time)
                explored_set = explored_set | explored_set_in
                if print_enable == True:
                	print('IDS algorithm path')
                print_path(cur_state , len(explored_set) , all_states , print_enable)
                return True , all_states , explored_set;
            continue
        if cur_depth == max_depth:
            continue

        bag_cnt = 0
        for i in range(k):
            if cur_state.bag_balls[i] == 1:
                bag_cnt += 1

        cur_pos = cur_state.pos

        #pick up
        if ((cur_pos.x , cur_pos.y) in balls_start.keys()) and bag_cnt + 1 <= c:
            ball_id = balls_start[(cur_pos.x , cur_pos.y)]
            if cur_state.bag_balls[ball_id] == 0:
                new_state_bag_balls = cur_state.bag_balls.copy()
                new_state_bag_balls[ball_id] = 1

                new_state = State(cur_state.pos , cur_state.cor_balls , new_state_bag_balls)

                if new_state not in mark.keys():
                    frontier_set.append((new_state , cur_depth + 1))
                    new_state.par = cur_state
                    mark[new_state] = True
                    depth[new_state] = cur_depth + 1
                elif cur_depth + 1 < depth[new_state]:
                    frontier_set.append((new_state , cur_depth + 1))
                    new_state.par = cur_state
                    depth[new_state] = cur_depth + 1                    

        for i in range(4):
            new_x = cur_pos.x + ROW_MOVES[i]
            new_y = cur_pos.y + COL_MOVES[i]
            new_pos = Point(new_x , new_y)

            if is_valid_pos(new_pos , n , m , table) == True:
                new_state_bag_balls = cur_state.bag_balls.copy()
                new_state_cor_balls = cur_state.cor_balls

                if (new_pos.x , new_pos.y) in balls_end.keys():
                    ball_id = balls_end[(new_pos.x , new_pos.y)]
                    if new_state_bag_balls[ball_id] == 1:
                        new_state_bag_balls[ball_id] = 2
                        new_state_cor_balls += 1
                
                new_state = State(new_pos , new_state_cor_balls , new_state_bag_balls)

                if new_state not in mark.keys():
                    frontier_set.append((new_state , cur_depth + 1))
                    new_state.par = cur_state
                    mark[new_state] = True
                    depth[new_state] = cur_depth + 1
                elif cur_depth + 1 < depth[new_state]:
                    frontier_set.append((new_state , cur_depth + 1))
                    new_state.par = cur_state
                    depth[new_state] = cur_depth + 1

    explored_set = explored_set | explored_set_in
    return False , all_states , explored_set;


def ids(n , m , start_state , end_state , c , k , start_time , print_enable):
	explored_set = set([])
	all_states = 0
	for max_depth in range(100):
		check , all_states , explored_set = dfs(n , m , start_state , end_state , c , k ,
												 max_depth , start_time , all_states , explored_set , print_enable)
		if check == True:
			break

def A_star_first(alpha , n , m , start_state , end_state , c , k , balls_start , 
					balls_end , table , start_time , print_enable):
    frontier_set = [(k * alpha , start_state)]
    explored_set = set([])
    mark = {}
    dist = {}

    all_states = 0

    heapq.heapify(frontier_set)
    dist[start_state] = k * alpha
    mark[start_state] = True
    while True:
        cur_state_t = heapq.heappop(frontier_set)
        all_states += 1
        f_cur_state = cur_state_t[0]
        cur_state = cur_state_t[1]

        explored_set.add(cur_state)

        if cur_state == end_state:
            end_time = time.time()
            execution_times.append(end_time - start_time)
            if alpha == 1 and print_enable == True:
            	print('First A* algorithm path')
            elif print_enable == True:
            	print('Weighted A* algorithm with alpha = ' + str(alpha))
            print_path(cur_state , len(explored_set) , all_states , print_enable)
            break

        bag_cnt = 0
        for i in range(k):
            if cur_state.bag_balls[i] == 1:
                bag_cnt += 1

        cur_pos = cur_state.pos

        #pick up
        if ((cur_pos.x , cur_pos.y) in balls_start.keys()) and bag_cnt + 1 <= c:
            ball_id = balls_start[(cur_pos.x , cur_pos.y)]
            if cur_state.bag_balls[ball_id] == 0:
                new_state_bag_balls = cur_state.bag_balls.copy()
                new_state_bag_balls[ball_id] = 1

                new_state = State(cur_state.pos , cur_state.cor_balls , new_state_bag_balls)

                if new_state not in mark.keys():
                    heapq.heappush(frontier_set , (f_cur_state + 1 , new_state))
                    dist[new_state] = f_cur_state + 1
                    new_state.par = cur_state
                    mark[new_state] = True
                elif f_cur_state + 1 < dist[new_state]:
                    heapq.heappush(frontier_set , (f_cur_state + 1 , new_state))
                    dist[new_state] = f_cur_state + 1
                    new_state.par = cur_state                    


        for i in range(4):
            new_x = cur_pos.x + ROW_MOVES[i]
            new_y = cur_pos.y + COL_MOVES[i]
            new_pos = Point(new_x , new_y)

            if is_valid_pos(new_pos , n , m , table) == True:
                new_state_bag_balls = cur_state.bag_balls.copy()
                new_state_cor_balls = cur_state.cor_balls

                check = False
                if (new_pos.x , new_pos.y) in balls_end.keys():
                    ball_id = balls_end[(new_pos.x , new_pos.y)]
                    if new_state_bag_balls[ball_id] == 1:
                        new_state_bag_balls[ball_id] = 2
                        new_state_cor_balls += 1
                        check = True
                
                f_new_state = f_cur_state + 1
                if check == True:
                    f_new_state -= 1 * alpha

                new_state = State(new_pos , new_state_cor_balls , new_state_bag_balls)
                if new_state not in mark.keys():
                    heapq.heappush(frontier_set , (f_new_state , new_state))
                    dist[new_state] = f_new_state
                    new_state.par = cur_state
                    mark[new_state] = True
                elif f_new_state < dist[new_state]:
                    heapq.heappush(frontier_set , (f_new_state , new_state))
                    dist[new_state] = f_new_state
                    new_state.par = cur_state                    


def distance(a , b):
    x_dif = a.x - b.x
    y_dif = a.y - b.y
    return math.sqrt(x_dif * x_dif + y_dif * y_dif)


def A_star_second(n , m , start_state , end_state , c , k , balls_start ,
					 balls_end , table , start_time , print_enable):
    frontier_set = [(distance(start_state.pos , end_state.pos) , start_state)]
    explored_set = set([])
    mark = {}
    dist = {}

    all_states = 0

    heapq.heapify(frontier_set)
    mark[start_state] = True
    dist[start_state] = distance(start_state.pos , end_state.pos)

    while True:
        cur_state_t = heapq.heappop(frontier_set)
        all_states += 1
        f_cur_state = cur_state_t[0]
        cur_state = cur_state_t[1]

        cur_real_cost = f_cur_state - distance(cur_state.pos , end_state.pos)

        explored_set.add(cur_state)

        if cur_state == end_state:
            end_time = time.time()
            execution_times.append(end_time - start_time)
            if print_enable == True:
            	print('Second A* algortihm path')
            print_path(cur_state , len(explored_set) , all_states , print_enable)
            break

        bag_cnt = 0
        for i in range(k):
            if cur_state.bag_balls[i] == 1:
                bag_cnt += 1

        cur_pos = cur_state.pos

        #pick up
        if ((cur_pos.x , cur_pos.y) in balls_start.keys()) and bag_cnt + 1 <= c:
            ball_id = balls_start[(cur_pos.x , cur_pos.y)]
            if cur_state.bag_balls[ball_id] == 0:
                new_state_bag_balls = cur_state.bag_balls.copy()
                new_state_bag_balls[ball_id] = 1

                new_state = State(cur_state.pos , cur_state.cor_balls , new_state_bag_balls)

                if new_state not in mark.keys():
                    heapq.heappush(frontier_set , (cur_real_cost + 1 + distance(new_state.pos , end_state.pos) , new_state))
                    dist[new_state] = cur_real_cost + 1 + distance(new_state.pos , end_state.pos)
                    new_state.par = cur_state
                    mark[new_state] = True
                elif cur_real_cost + 1 + distance(new_state.pos , end_state.pos) < dist[new_state]:
                    heapq.heappush(frontier_set , (cur_real_cost + 1 + distance(new_state.pos , end_state.pos) , new_state))
                    new_state.par = cur_state                    
                

        for i in range(4):
            new_x = cur_pos.x + ROW_MOVES[i]
            new_y = cur_pos.y + COL_MOVES[i]
            new_pos = Point(new_x , new_y)

            if is_valid_pos(new_pos , n , m , table) == True:
                new_state_bag_balls = cur_state.bag_balls.copy()
                new_state_cor_balls = cur_state.cor_balls

                if (new_pos.x , new_pos.y) in balls_end.keys():
                    ball_id = balls_end[(new_pos.x , new_pos.y)]
                    if new_state_bag_balls[ball_id] == 1:
                        new_state_bag_balls[ball_id] = 2
                        new_state_cor_balls += 1

                new_state = State(new_pos , new_state_cor_balls , new_state_bag_balls)
                if new_state not in mark.keys():
                    heapq.heappush(frontier_set , (cur_real_cost + 1 + distance(new_state.pos , end_state.pos) , new_state))
                    dist[new_state] = cur_real_cost + 1 + distance(new_state.pos , end_state.pos)
                    new_state.par = cur_state
                    mark[new_state] = True
                elif cur_real_cost + 1 + distance(new_state.pos , end_state.pos) < dist[new_state]:
                    heapq.heappush(frontier_set , (cur_real_cost + 1 + distance(new_state.pos , end_state.pos) , new_state))
                    dist[new_state] = cur_real_cost + 1 + distance(new_state.pos , end_state.pos)
                    new_state.par = cur_state

test_number = input('Enter the test number : ')
test_file_path = 'Tests/' + test_number + '.txt'

test_file = open(test_file_path , 'r')
n , m = map(int , test_file.readline().split())

start_x , start_y = map(int , test_file.readline().split())
start_point = Point(start_x , start_y)

end_x , end_y = map(int , test_file.readline().split())
end_point = Point(end_x , end_y)

c = int(test_file.readline())
k = int(test_file.readline())

for i in range(k):
    ball_start_x , ball_start_y , ball_end_x , ball_end_y = map(int , test_file.readline().split())

    balls_start[(ball_start_x , ball_start_y)] = i
    balls_end[(ball_end_x , ball_end_y)] = i

for i in range(n):
    cur_row = list(test_file.readline().split())
    table.append(cur_row)

start_state = State(start_point , 0 , [0 for i in range(k)])
end_state = State(end_point , k , [2 for i in range(k)])

test_file.close()

for i in range(3):
	print_enable = (i == 2)
	bfs(n , m , start_state , end_state , c , k , balls_start , balls_end , table , time.time() , print_enable)

	ids(n , m , start_state , end_state , c , k , time.time() , print_enable)

	A_star_first(1 , n , m , start_state , end_state , c , k , balls_start ,
									 balls_end , table , time.time() , print_enable)

	A_star_second(n , m , start_state , end_state , c , k , balls_start , 
									 balls_end , table , time.time() , print_enable)

	A_star_first(2 , n , m , start_state , end_state , c , k , balls_start ,
									 balls_end , table , time.time() , print_enable)

	A_star_first(5 , n , m , start_state , end_state , c , k , balls_start ,
									 balls_end , table , time.time() , print_enable)

for i in range(6):
	avg_exec_time = 0
	print(str(execution_times[i]) + '|' + str(execution_times[i + 6]) + '|' + str(execution_times[i + 12]))
	avg_exex_time = (execution_times[i] + execution_times[i + 6] + execution_times[i + 12]) / 3
	print('---->' + names[i] + ' avg time = ' + str(avg_exex_time))








