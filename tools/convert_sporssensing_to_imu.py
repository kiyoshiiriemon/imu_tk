# スポーツセンシング社のワイヤレスモーションセンサにより取得したcsvを.imuファイルに変換します

import sys, os, math, glob
import codecs
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
from datetime import datetime
from collections import defaultdict

data_hdrs_hr = ['シーケンス番号', '加速度X', '加速度Y', '加速度Z', '角速度X', '角速度Y', '角速度Z', 'vx', 'vy', 'vz', 'px', 'py', 'pz', 'qw', 'qx', 'qy', 'qz']
data_scales = [1, -9.80665, -9.80665, -9.80665, math.pi/180, math.pi/180, math.pi/180, 1, 1, 1, 1, 1, 1, 1., 1., 1., 1.]

def plot_logfile(fname, outfname):
    with open(fname) as f:
        data = np.loadtxt(f, delimiter='\t')
    fig = plt.figure(figsize=(8,4), dpi=300)
    ax = fig.add_subplot(2, 1, 1)
    #print('accx: ' + str(len(accx)))
    lw = 0.5
    ax.plot( data[:,0], data[:,1], color='r', label='accX', linewidth=lw )
    ax.plot( data[:,0], data[:,2], color='g', label='accY', linewidth=lw )
    ax.plot( data[:,0], data[:,3], color='b', label='accZ', linewidth=lw )
    #ax.legend()
    ax.legend(bbox_to_anchor=(1.05, 1), loc='upper left', borderaxespad=0)

    bx = fig.add_subplot(2, 1, 2)
    bx.plot( data[:,0], data[:,4], color='r', label='gyrX', linewidth=lw )
    bx.plot( data[:,0], data[:,5], color='g', label='gyrY', linewidth=lw )
    bx.plot( data[:,0], data[:,6], color='b', label='gyrZ', linewidth=lw )
    #bx.legend()
    lgd = bx.legend(bbox_to_anchor=(1.05, 1), loc='upper left', borderaxespad=0)

    wthre = 0.2
    athre = 5
    st = data[0, 0]
    et = data[0,-1]
    it = st
    for i in range(data.shape[0]):
        w = np.linalg.norm(data[i,4:6])
        if w > wthre:
            st = data[i,0] - 0.2
            break

    for i in list(reversed(range(data.shape[0]))):
        w = np.linalg.norm(data[i,4:6])
        if w > wthre:
            et = data[i,0] + 0.2
            break

    adiff = np.zeros(data.shape[0])
    for i in range(data.shape[0]-1):
        a0 = np.linalg.norm(data[i,1:3])
        a1 = np.linalg.norm(data[i+1,1:3])
        d = math.fabs(a1-a0)
        t = data[i,0]
        if t < st + 1 or t > et - 1:
            d = 0 # discard
        adiff[i] = d
    it = data[adiff.argmax(), 0]

    ax.axvline(x=st, linewidth=0.1)
    ax.axvline(x=et, linewidth=0.1)
    ax.axvline(x=it, linewidth=0.1)
    bx.axvline(x=st, linewidth=0.1)
    bx.axvline(x=et, linewidth=0.1)
    #ax.plot(time, accy, linestyle='--', color='y', label='accy')
    ax.title.set_text(os.path.basename(fname))
    plt.savefig(outfname, bbox_extra_artists=(lgd,), bbox_inches='tight')
    #plt.plot([1, 2, 3])
    #plt.show()

def main(fname, data_hdrs, suffix):
    count = 0
    indices = {}
    means = {}
    leaf, ext = os.path.splitext(fname)
    outfname = leaf + suffix
    print(outfname)
    of = open(outfname, 'w')
    f = codecs.open(fname, 'r', 'shift_jis')
    linecnt = 0
    found_header = False
    for line in f:
        linecnt += 1

        if line.startswith('計測開始時刻'):
            items = line.split(',')
            datetime_formats = ['%Y/%m/%d %H:%M', '%y/%m/%d %H:%M']
            for format in datetime_formats:
                try:
                    datetime0 = datetime.strptime(items[1] + ' ' + items[2], format)
                    if datetime0:
                        break
                except ValueError:
                    print('testing datetime format')

            #print(datetime0)
            starttime = datetime0.timestamp()
            print(f'{datetime0} -> {starttime}')
        if line.startswith('サンプリング周波数'):
            items = line.split(',')
            datafreq = float(items[1])
            print(f'data freq {datafreq}')

        if line.startswith('シーケンス番号'):
            found_header = True
            
        if not found_header:
            continue
        #print('process: ' + line)
        items = line.split(',')
        if len(indices) == 0:
            # read header
            for h in data_hdrs:
                indices[h] = items.index(h) if h in items else -1
                print(f'{h}: {indices[h]}')
                means[h] = 0
            print(indices)
            data_sample = defaultdict(lambda: 0)
        else:
            for i, h in enumerate(data_hdrs):
                if indices[h] < 0:
                    continue
                elem = float(items[indices[h]]) * data_scales[i]
                if elem:
                    data_sample[h] = elem
            #if len(data_sample) == len(data_hdrs):
            st = ''
            for i, h in enumerate(data_hdrs):
                if i == 0:
                    time_sec = int(data_sample[h]) / datafreq + starttime
                    sec = math.floor(time_sec)
                    nsec = int((time_sec - sec) * 1e9)
                    st += str(sec) + '\t' + str(nsec) + '\t'
                else:
                    means[h] += float(data_sample[h])
                    st += str(float(data_sample[h])) + '\t'
            #of.write(st + '0\t0\t0\t0\t0\t0\t1\t0\t0\t0\n')
            of.write(st + '\n')
            data_sample = defaultdict(lambda: 0)
            count += 1

    return outfname

if __name__ == "__main__":
    if len(sys.argv) <= 1:
        print("need an argument")
    else:
        for i in range(1, len(sys.argv)):
            main(sys.argv[i], data_hdrs_hr, '_ss.imu')
