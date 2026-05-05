from google.colab.patches import cv2_imshow
from IPython.display import clear_output

import numpy as np
import cv2
import math
import base64
import time
from IPython.display import display, HTML, clear_output

# 全域 handle 用於更新畫面
display_handle = None

def rotatePoint(x, y, cx, cy, a):
    x_shifted = x - cx 
    y_shifted = y - cy
    cos_a = math.cos(a)
    sin_a = math.sin(a)
    return (cx + int(x_shifted * cos_a - y_shifted * sin_a), 
            cy + int(x_shifted * sin_a + y_shifted * cos_a))

def coordinateMapping(x, y, maxY=512):
    return (int(x), int(maxY - y))

def array_to_base64(img):
    _, buffer = cv2.imencode('.jpg', img)
    return base64.b64encode(buffer).decode('utf-8')

class Simulation():
    def __init__(self, ox=400, oy=256, scrsize=512):
        self.SCREEN_SIZE = scrsize
        self.SCREEN_CENTER_X = int(self.SCREEN_SIZE / 2)
        self.parkingLotWidth = 250
        self.parkingLotHeight = 150
        #停車格左右邊界
        self.Tip1_X = self.SCREEN_CENTER_X + int(0.5*self.parkingLotWidth)
        self.Tip2_X = self.SCREEN_CENTER_X - int(0.5*self.parkingLotWidth)
        
        self.theta = 0  # 車身方向角
        self.phi = 0.0  # 前輪轉角
        self.velocity = 0.0 # 速度
        self.l_axis = 150  # 軸距（前後輪距離）
        self.carWidth = 80
        self.WheelLength = 20
        self.ext = 10    # 車頭尾延伸
        self.Xr, self.Yr = ox, oy # 後輪位置
        self.Xf, self.Yf = 0, 0  # 前輪位置
    #後輪到停車格右邊距離
    def getTheta(self): return self.theta
    def getRearToLotTip1(self): return (self.Xr - self.Tip1_X)
    def getRearToLotTip2(self): return (self.Xr - self.Tip2_X)

    def drawSimulation(self, phi, velocity):
        global display_handle
        self.phi = phi
        self.velocity = velocity 

        # 1. 物理運動學計算
        dTheta = self.velocity * math.sin(self.phi) / self.l_axis#車子轉動角速度
        self.theta += dTheta
        #後輪移動
        self.Xr += self.velocity * math.cos(self.theta) * math.cos(self.phi)
        self.Yr += self.velocity * math.sin(self.theta) * math.cos(self.phi)
        #前輪位置
        self.Xf = self.Xr + self.l_axis * math.cos(self.theta)
        self.Yf = self.Yr + self.l_axis * math.sin(self.theta)

        # 2. 建立畫布
        img = np.zeros((self.SCREEN_SIZE, self.SCREEN_SIZE + 300, 3), np.uint8)
        img.fill(200)

        # 畫停車格
        cv2.rectangle(img, (self.SCREEN_CENTER_X - 125, self.SCREEN_SIZE - 150),
                      (self.SCREEN_CENTER_X + 125, self.SCREEN_SIZE - 1), (255, 255, 255), 5)

        # 3. 計算車身頂點
        carCenterX, carCenterY = (self.Xr + self.Xf) / 2, (self.Yr + self.Yf) / 2
        #車身四個角（未旋轉
        pLeftX, pRightX = -0.5 * self.l_axis - self.ext, 0.5 * self.l_axis + self.ext
        pTopY, pBottomY = 0.5 * self.carWidth, -0.5 * self.carWidth
        
        car_pts = []
        #每個角做旋轉
        for x, y in [(pLeftX, pTopY), (pRightX, pTopY), (pRightX, pBottomY), (pLeftX, pBottomY)]:
            px, py = rotatePoint(x + carCenterX, y + carCenterY, carCenterX, carCenterY, self.theta)
            car_pts.append(coordinateMapping(px, py))

        cv2.polylines(img, [np.array(car_pts, np.int32)], True, (255, 0, 0), 3)

        # 4. 畫輪胎 (重點修正處)
        # 後輪 (固定方向)
        for y_off in [pTopY, pBottomY]:
            w1x, w1y = rotatePoint(pLeftX + carCenterX, y_off + carCenterY, carCenterX, carCenterY, self.theta)
            w2x, w2y = rotatePoint(pLeftX + self.WheelLength + carCenterX, y_off + carCenterY, carCenterX, carCenterY, self.theta)
            cv2.line(img, coordinateMapping(w1x, w1y), coordinateMapping(w2x, w2y), (0, 100, 0), 5)

        # 前輪 (隨 phi 轉向)
        for y_off in [pTopY, pBottomY]:
            # 先找到前輪中心在車身上的位置
            fw_center_x, fw_center_y = rotatePoint(pRightX - self.WheelLength/2 + carCenterX, y_off + carCenterY, carCenterX, carCenterY, self.theta)
            # 計算輪胎兩端 (考慮 theta + phi)
            w1x = fw_center_x + (self.WheelLength/2) * math.cos(self.theta + self.phi)
            w1y = fw_center_y + (self.WheelLength/2) * math.sin(self.theta + self.phi)
            w2x = fw_center_x - (self.WheelLength/2) * math.cos(self.theta + self.phi)
            w2y = fw_center_y - (self.WheelLength/2) * math.sin(self.theta + self.phi)
            cv2.line(img, coordinateMapping(w1x, w1y), coordinateMapping(w2x, w2y), (0, 255, 0), 5)

        # 畫軸線
        cv2.line(img, coordinateMapping(self.Xr, self.Yr), coordinateMapping(self.Xf, self.Yf), (0, 0, 255), 2)

        # 5. 顯示邏輯
        b64_str = array_to_base64(img)
        html_code = f'<img src="data:image/jpeg;base64,{b64_str}" style="width:600px;">'
        if display_handle is None:
            display_handle = display(HTML(html_code), display_id=True)
        else:
            display_handle.update(HTML(html_code))

# 執行主程式
if __name__ == '__main__':
    display_handle = None 
    clear_output(wait=True)
    sim = Simulation(400, 250)
    phi, v, phase, stop = 0, 0, 1, False
    
    while not stop:
        sim.drawSimulation(phi, v)
        if phase == 1:
            v -= 0.1
            if abs(sim.getRearToLotTip1()) < 10: phase = 2
        elif phase == 2:
            phi, v = -math.pi/2, -5
            if abs(sim.getTheta() - math.pi/4) < 0.1: phase = 3
        elif phase == 3:
            phi, v = 0, -1.5
            if abs(sim.getRearToLotTip2()) < 100: phase = 4
        elif phase == 4:
            phi, v = math.pi/4, -3
            if abs(sim.getTheta() - 0) < 0.1: phase = 5
        elif phase == 5:
          if abs(sim.getTheta()) > 0.02:
            if sim.getTheta() > 0:
              phi = -0.25
            else:
              phi = 0.25

            v = 2   # 慢慢往前修正

          # 更嚴格一點，角度接近 0 才停
          else:
            phi, v = 0, 0
            sim.drawSimulation(phi, v)  # 讓輪胎回正
            stop = True
        time.sleep(0.05)

  
