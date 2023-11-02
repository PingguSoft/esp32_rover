from utils import *
import matplotlib.pyplot as plt
from icp import *
import cv2

plt.switch_backend('agg')

if __name__ == "__main__":
    lidar, lidar_data, name = ReadData(0)
    print(f'lidar_data={lidar_data.shape}')

    start = 0
    end, _ = lidar_data.shape
    gap = 5

    pose = [0, 0, 0]

    traj = []

    for i in range(start, end, gap):
        try:
            scan_before_local = lidar.ReadAScan(lidar_data, i, 60)  # (N , 2)
            scan_current_local = lidar.ReadAScan(lidar_data, i+gap, 60)  # (N' ,2)

            scan_before_global = localToGlobal(pose, scan_before_local)
            scan_current_global = localToGlobal(pose, scan_current_local)

            fig = plt.figure(figsize=(8, 8))
            plt.clf()
            plt.xlim([-80, 80])
            plt.ylim([-80, 80])

            plt.plot(pose[0], pose[1], 'o', color='blue', markersize=3)
            traj.append(pose)

            traj_array = np.array(traj)
            plt.plot(traj_array[:, 0], traj_array[:, 1], color='black')

            T = icp(scan_current_global, scan_before_global)

            pose_T = v2t(pose)
            pose = t2v(np.copy(np.dot(T, pose_T)))

            frame = np.ones((3, scan_current_global.shape[0]))
            frame[:2, :] = scan_current_global.T
            result = (T @ frame)[:2, :].T

            plt.plot(result[:, 0], result[:, 1], 'o', markersize=1, color='red')
            # plt.savefig(f'result/{i:08d}.png', dpi=300)
            fig.canvas.draw()
            img = np.frombuffer(fig.canvas.tostring_rgb(), dtype=np.uint8)
            img = img.reshape(fig.canvas.get_width_height()[::-1] + (3,))
            plt.close(fig)
            img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
            cv2.imshow('pose', img)
            cv2.waitKey(10)
        except KeyboardInterrupt:
            break
