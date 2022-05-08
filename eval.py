import argparse
import os
import matplotlib.pyplot as plt

parser = argparse.ArgumentParser(description='CENG 4120 Clock Tree Synthesis Evaluator')
parser.add_argument('--input', required=True)
parser.add_argument('--output', default='')
parser.add_argument('--plot_only', default=False)
parser.add_argument('--fig', default='')
parser.add_argument('--plot', default='false')
args = parser.parse_args()

max_time = 0
max_load = 0
size = 0
capacity = 0
num_pins = 0
num_taps = 0

pins = []
taps = []

print('Reading {}...'.format(args.input))
with open(args.input) as f:
    line = f.readline()
    while len(line) > 0:
        data = line.strip().split(' ')
        if data[0] == 'MAX_RUNTIME':
            max_time = int(data[1])
        elif data[0] == 'MAX_LOAD':
            max_load = int(data[1])
        elif data[0] == 'GRID_SIZE':
            size = int(data[1])
        elif data[0] == 'CAPACITY':
            capacity = int(data[1])
        elif data[0] == 'PINS':
            num_pins = int(data[1])
            for i in range(num_pins):
                data = f.readline().strip().split(' ')
                pins.append([int(data[2]), int(data[3])])
        elif data[0] == 'TAPS':
            num_taps = int(data[1])
            for i in range(num_taps):
                data = f.readline().strip().split(' ')
                taps.append([int(data[2]), int(data[3])])
        
        line = f.readline()

assigned = [False for i in range(num_pins)]
tap_pins = [[] for i in range(num_taps)]
tap_edges = [[] for i in range(num_taps)]
score = 0

if args.plot_only:
    if args.output != '':
        print('Reading {}...'.format(args.output))
        with open(args.output) as f:
            line = f.readline()
            while len(line) > 0:
                data = line.strip().split(' ')
                if data[0] == 'TAP':
                    tap = int(data[1])
                elif data[0] == 'PINS':
                    num_tap_pins = int(data[1])
                    for i in range(num_tap_pins):
                        data = f.readline().strip().split(' ')
                        tap_pins[tap].append(int(data[1]))
                elif data[0] == 'ROUTING':
                    num_edges = int(data[1])
                    for i in range(num_edges):
                        data = f.readline().strip().split(' ')
                        tap_edges[tap].append([
                            (int(data[1]), int(data[2])),
                            (int(data[3]), int(data[4]))
                        ])
                line = f.readline()
    length = 0
    for t in range(num_taps):
        used = [[[0 for y in range(size)] for x in range(size)] for d in range(2)]
        for edge in tap_edges[t]:
            if edge[0][0] == edge[1][0]:
                x = edge[0][0]
                l = min(edge[0][1], edge[1][1])
                h = max(edge[0][1], edge[1][1])
                for y in range(l, h):
                    if used[1][x][y] == 0:
                        used[1][x][y] = 1
                        length += 1
            elif edge[0][1] == edge[1][1]:
                y = edge[0][1]
                l = min(edge[0][0], edge[1][0])
                h = max(edge[0][0], edge[1][0])
                for x in range(l, h):
                    if used[0][x][y] == 0:
                        used[0][x][y] = 1
                        length += 1
    print('Total length = {}.'.format(length))
    
    # plot_name = args.input.replace('.in', '.png')
    design = args.input[args.input.rfind('/')+1:].replace('.in', '.png')
    plot_name = os.path.join(args.fig, design)
    # print(args.fig)
    # print(design)
    # print(plot_name)
    # exit(1)
    print('Plotting...')
    cmap = plt.get_cmap('nipy_spectral')
    offset_width = min((num_taps - 1) * 0.05, 0.5)
    min_offset = (1 - offset_width) / 2
    fig = plt.figure(figsize=(size/5, size/5))
    for t in range(num_taps):
        color = cmap(t/num_taps)
        offset = min_offset + offset_width * (t / max(1,num_taps-1))
        # plot edges
        if args.output != '':
            for edge in tap_edges[t]:
                plt.plot(
                    [edge[0][0] + offset, edge[1][0] + offset], 
                    [edge[0][1] + offset, edge[1][1] + offset],
                    color=color
                )
        # plot taps
        plt.scatter([taps[t][0] + offset], [taps[t][1] + offset], marker='o', 
                    s=20, color=color, linewidths=0)
        # plot pins
        xs = []
        ys = []
        s = []
        for p in tap_pins[t]:
            xs.append(pins[p][0] + offset)
            ys.append(pins[p][1] + offset)
            s.append(10)
        plt.scatter(xs, ys, marker='s', s=s, color=color, linewidths=0)
    # plot unassigned pins 
    xs = []
    ys = []
    s = []
    for i, pin in enumerate(pins):
        if assigned[i]:
            continue
        xs.append(pin[0] + 0.5)
        ys.append(pin[1] + 0.5)
        s.append(10)
    plt.scatter(xs, ys, marker='s', s=s, color='black', linewidths=0)
    # plot grids
    plt.xticks([x for x in range(size + 1)])
    plt.yticks([y for y in range(size + 1)])
    plt.gca().grid()
    # plt.legend()
    plt.savefig(plot_name)
    plt.show()
    
else:
    if args.output != '':
        print('Reading {}...'.format(args.output))
        with open(args.output) as f:
            line = f.readline()
            while len(line) > 0:
                data = line.strip().split(' ')
                if data[0] == 'TAP':
                    tap = int(data[1])
                elif data[0] == 'PINS':
                    num_tap_pins = int(data[1])
                    for i in range(num_tap_pins):
                        data = f.readline().strip().split(' ')
                        tap_pins[tap].append(int(data[1]))
                elif data[0] == 'ROUTING':
                    num_edges = int(data[1])
                    for i in range(num_edges):
                        data = f.readline().strip().split(' ')
                        tap_edges[tap].append([
                            (int(data[1]), int(data[2])),
                            (int(data[3]), int(data[4]))
                        ])
                line = f.readline()
        
        for t_pins in tap_pins:
            for pin in t_pins:
                if assigned[pin]:
                    print('Error: pin {} is assigned to more than one tap.'.format(pin))
                    exit(0)
                assigned[pin] = True
        for i, a in enumerate(assigned):
            if not a:
                print('Error: pin {} is not assigned to any tap.'.format(i))
                exit(0)
        print('Passed: Assignment Check.')
        
        length = 0
        tap_used = []
        tap_skew = []
        for t in range(num_taps):
            used = [[[0 for y in range(size)] for x in range(size)] for d in range(2)]
            for edge in tap_edges[t]:
                if edge[0][0] == edge[1][0]:
                    x = edge[0][0]
                    l = min(edge[0][1], edge[1][1])
                    h = max(edge[0][1], edge[1][1])
                    for y in range(l, h):
                        if used[1][x][y] == 0:
                            used[1][x][y] = 1
                            length += 1
                elif edge[0][1] == edge[1][1]:
                    y = edge[0][1]
                    l = min(edge[0][0], edge[1][0])
                    h = max(edge[0][0], edge[1][0])
                    for x in range(l, h):
                        if used[0][x][y] == 0:
                            used[0][x][y] = 1
                            length += 1
                else:
                    print('Error: edge {} {} {} {} is neither horizontal or vertical.'
                        .format(edge[0][0], edge[0][1], edge[1][0], edge[1][1]))
                    exit(0)
                
                if min(edge[0][0], edge[0][1], edge[1][0], edge[1][1]) < 0 or \
                max(edge[0][0], edge[0][1], edge[1][0], edge[1][1]) >= size:
                    print('Error: edge {} {} {} {} is out of boundary.'
                        .format(edge[0][0], edge[0][1], edge[1][0], edge[1][1]))
                    exit(0)
            tap = taps[t]
            arrival_time = [[1e9 for y in range(size)] for x in range(size)]
            
            to_update = [[tap, 0, used, arrival_time]]
            i = 0
            while i < len(to_update):
                cur = to_update[i][0]
                time = to_update[i][1]
                used = to_update[i][2]
                arrival_time = to_update[i][3]
                if time < arrival_time[cur[0]][cur[1]]:
                    arrival_time[cur[0]][cur[1]] = time
                    if cur[0] > 0 and used[0][cur[0] - 1][cur[1]]:
                        to_update.append([[cur[0] - 1, cur[1]], time+1, used, arrival_time])
                    if cur[0] < size - 1 and used[0][cur[0]][cur[1]]:
                        to_update.append([[cur[0] + 1, cur[1]], time+1, used, arrival_time])
                    if cur[1] > 0 and used[1][cur[0]][cur[1] - 1]:
                        to_update.append([[cur[0], cur[1] - 1], time+1, used, arrival_time])
                    if cur[1] < size - 1 and used[1][cur[0]][cur[1]]:
                        to_update.append([[cur[0], cur[1] + 1], time+1, used, arrival_time])
                i += 1
                
            min_time = 1e9
            max_time = 0
            for p in tap_pins[t]:
                pin = pins[p]
                if arrival_time[pin[0]][pin[1]] == 1e9:
                    print('Error: pin {} is not connected to tap {}'.format(p, t))
                    exit(0)
                min_time = min(min_time, arrival_time[pin[0]][pin[1]])
                max_time = max(max_time, arrival_time[pin[0]][pin[1]])
            tap_skew.append(max_time - min_time)
            
            tap_used.append(used)
        print('Passed: Connectivity Check.')
        
        overall_used = [[[0 for y in range(size)] for x in range(size)] for d in range(2)]
        max_used = 0
        max_used_edge = []
        for t in range(num_taps):
            for d in range(2):
                for x in range(size):
                    for y in range(size):
                        overall_used[d][x][y] += tap_used[t][d][x][y]
                        if overall_used[d][x][y] > max_used:
                            max_used = overall_used[d][x][y]
                            max_used_edge = [d, x, y]
        if max_used > capacity:
            low_end = (max_used_edge[1], max_used_edge[2])
            if max_used_edge[0] == 0:
                high_end = (max_used_edge[1] + 1, max_used_edge[2])
            else:
                high_end = (max_used_edge[1], max_used_edge[2] + 1)
            print('Error: grid edge {} to {} has exceeded the maximum capacity.'.format(low_end, high_end))
            # exit(0)
        else:
            print('Passed: Capacity Check. (40)')
            score += 40
            
        for i in range(num_taps):
            if len(tap_pins[i]) > max_load:
                print('Error: tap {} exceeds the maximum load.'.format(i))
                exit(0)
        
        if score >= 40:
            print('Passed: Load Check. (20)')
            score += 20
        else:
            print('Passed: Load Check.')
            
            
        zero_skew = True
        for t in range(num_taps):
            if tap_skew != 0:
                zero_skew = False
            print('Tap {} skew = {}'.format(t, tap_skew[t]))
        if zero_skew:
            print('All trees have 0 skews! (10)')
            score += 10
            
        print('Total length = {}.'.format(length))
        if score < 60:
            print('Score = {}'.format(score))
        else:
            print('Score = {}+'.format(score))
            print('(Tree length score will be determined later)')
        
                

    if args.plot.lower() == 'true':
        # plot_name = args.input.replace('.in', '.png')
        design = args.input[args.input.rfind('/')+1:].replace('.in', '.png')
        plot_name = os.path.join(args.fig, design)
        # print(args.fig)
        # print(design)
        # print(plot_name)
        # exit(1)
        print('Plotting...')
        cmap = plt.get_cmap('nipy_spectral')
        offset_width = min((num_taps - 1) * 0.05, 0.5)
        min_offset = (1 - offset_width) / 2
        fig = plt.figure(figsize=(size/5, size/5))
        for t in range(num_taps):
            color = cmap(t/num_taps)
            offset = min_offset + offset_width * (t / max(1,num_taps-1))
            # plot edges
            if args.output != '':
                for edge in tap_edges[t]:
                    plt.plot(
                        [edge[0][0] + offset, edge[1][0] + offset], 
                        [edge[0][1] + offset, edge[1][1] + offset],
                        color=color
                    )
            # plot taps
            plt.scatter([taps[t][0] + offset], [taps[t][1] + offset], marker='o', 
                        s=20, color=color, linewidths=0)
            # plot pins
            xs = []
            ys = []
            s = []
            for p in tap_pins[t]:
                xs.append(pins[p][0] + offset)
                ys.append(pins[p][1] + offset)
                s.append(10)
            plt.scatter(xs, ys, marker='s', s=s, color=color, linewidths=0)
        # plot unassigned pins 
        xs = []
        ys = []
        s = []
        for i, pin in enumerate(pins):
            if assigned[i]:
                continue
            xs.append(pin[0] + 0.5)
            ys.append(pin[1] + 0.5)
            s.append(10)
        plt.scatter(xs, ys, marker='s', s=s, color='black', linewidths=0)
        # plot grids
        plt.xticks([x for x in range(size + 1)])
        plt.yticks([y for y in range(size + 1)])
        plt.gca().grid()
        # plt.legend()
        plt.savefig(plot_name)
        plt.show()
    
        