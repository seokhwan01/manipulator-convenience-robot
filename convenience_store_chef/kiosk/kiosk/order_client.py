import tkinter as tk
from tkinter import messagebox
from PIL import Image, ImageTk
import threading
import time
from queue import Queue

# ROS2 관련 모듈
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from convenience_store_chef_interfaces.action import OrderService

import os
from ament_index_python.packages import get_package_share_directory

# 초기 재고 설정
stock = {
    "김밥": 2,
    "삼각김밥": 2,
    "라면": 1
}

prices = {
    "김밥": 2000,
    "삼각김밥": 1200,
    "라면": 3500
}

quantities = {item: 0 for item in stock}

class OrderActionClient(Node):
    def __init__(self):
        super().__init__('order_action_client')
        self._client = ActionClient(self, OrderService, '/order_service')
        self.order_queue = Queue()
        self._processing = False

        self._spin_thread = threading.Thread(target=rclpy.spin, args=(self,), daemon=True)
        self._spin_thread.start()

    def enqueue_order(self, items, qtys, feedback_callback, result_callback):
        self.order_queue.put((items, qtys, feedback_callback, result_callback))
        self._try_process_next()

    def _try_process_next(self):
        if self._processing or self.order_queue.empty():
            return

        self._processing = True
        items, qtys, feedback_cb, result_cb = self.order_queue.get()
        goal_msg = OrderService.Goal()
        goal_msg.items = items
        goal_msg.quantities = qtys

        self._client.wait_for_server()
        send_future = self._client.send_goal_async(goal_msg, feedback_callback=feedback_cb)

        def goal_response_cb(future):
            goal_handle = future.result()
            if not goal_handle.accepted:
                print("❌ Goal rejected")
                self._processing = False
                result_cb(None)
                self._try_process_next()
                return

            result_future = goal_handle.get_result_async()

            def result_received_cb(rf):
                result = rf.result().result
                result_cb(result)
                self._processing = False
                self._try_process_next()

            result_future.add_done_callback(result_received_cb)

        send_future.add_done_callback(goal_response_cb)

def main():
    rclpy.init()
    ros_node = OrderActionClient()

    root = tk.Tk()
    root.title("편의점 키오스크")
    root.geometry("1600x1200")

    for i in range(7):
        root.grid_rowconfigure(i, weight=1)
    for i in range(6):
        root.grid_columnconfigure(i, weight=1)

    def load_image(filename):
        try:
            img = Image.open(filename)
            img = img.resize((150, 150))
            return ImageTk.PhotoImage(img)
        except:
            return None
        
    path = get_package_share_directory('kiosk')
    images = {
        "김밥": load_image(os.path.join(path, 'resource', 'kimbap.png')),
        "삼각김밥": load_image(os.path.join(path, 'resource', '3_kimbap.png')),
        "라면": load_image(os.path.join(path, 'resource', 'ramen.png'))
    }

    qty_labels = {}
    stock_labels = {}

    def create_menu_item(name, row):
        tk.Label(root, text=f"{name} - {prices[name]}원", font=("Arial", 20)).grid(row=row, column=0, pady=20, sticky="w")
        if images[name]:
            tk.Label(root, image=images[name]).grid(row=row, column=1)
        tk.Button(root, text="-", font=("Arial", 18), width=3,
                  command=lambda: update_quantity(name, -1)).grid(row=row, column=2)
        qty_labels[name] = tk.Label(root, text="0", width=5, font=("Arial", 18))
        qty_labels[name].grid(row=row, column=3)
        tk.Button(root, text="+", font=("Arial", 18), width=3,
                  command=lambda: update_quantity(name, 1)).grid(row=row, column=4)
        stock_labels[name] = tk.Label(root, text=f"재고: {stock[name]}", fg="blue", font=("Arial", 16))
        stock_labels[name].grid(row=row, column=5)

    def update_quantity(name, delta):
        if delta == 1:
            if quantities[name] < stock[name]:
                quantities[name] += 1
            else:
                messagebox.showwarning("재고 부족", f"{name}의 재고가 부족합니다.")
        elif delta == -1 and quantities[name] > 0:
            quantities[name] -= 1
        qty_labels[name].config(text=str(quantities[name]))

    def place_order():
        selected_items = [(item, quantities[item]) for item in quantities if quantities[item] > 0] 
        #리스트 안 튜플형식
        #[("김밥", 2), ("라면", 1), ...]
        if not selected_items:
            messagebox.showinfo("알림", "하나 이상의 상품을 선택하세요.")
            return

        total = sum(prices[item] * qty for item, qty in selected_items)
        summary_lines = [f"{item} {qty}개 = {prices[item]*qty}원" for item, qty in selected_items]

        print("[🧾 주문 접수]", summary_lines)

        # ✅ UI는 바로 주문 완료처럼 보여줌
        for item, qty in selected_items:
            stock[item] -= qty
            stock_labels[item].config(text=f"재고: {stock[item]}")
        result_label.config(text=f"총 금액: {total}원\n주문이 접수되었습니다.")
        disable_buttons()
        threading.Thread(target=delayed_reset, daemon=True).start()

        # ✅ ROS2 액션 비동기 전송
        #딕셔너리로 합치고 정렬
        #딕셔너리 변환
        def custom_order(item_name):
         # 예: '김밥'은 1순위, '라면'은 2순위, '삼각김밥'은 3순위
         #사용자 정의 정렬 함수
            order = {
                "라면": 1,
                "김밥": 2,
                "삼각김밥": 3
            }
            return order.get(item_name, 99)  # 정의되지 않은 항목은 뒤로
        # 정렬된 튜플 리스트 생성
        sorted_items = sorted(selected_items, key=lambda x: custom_order(x[0]))

        item_names = [item for item, qty in sorted_items]
        item_qtys = [qty for item, qty in sorted_items]

        def feedback_cb(feedback_msg):
            print(f"[피드백] {feedback_msg.feedback.status}")

        def result_cb(result):
            if result is None:
                print("❌ 주문 실패: goal rejected or error")
            elif not result.success:
                print(f"❌ 주문 처리 실패: {result.message}")
            else:
                print(f"[✅ 주문 처리 완료] 서버 응답 메시지: {result.message}")

        ros_node.enqueue_order(item_names, item_qtys, feedback_cb, result_cb)

        for item in quantities:
            quantities[item] = 0
            qty_labels[item].config(text="0")

    def disable_buttons():
        order_button.config(state="disabled")
        for item in qty_labels:
            qty_labels[item].config(state="disabled")

    def delayed_reset():
        time.sleep(5)
        reset_all()
        order_button.config(state="normal")
        for item in qty_labels:
            qty_labels[item].config(state="normal")

    def order_more():
        more_clicked[0] = True
        for item in quantities:
            quantities[item] = 0
            qty_labels[item].config(text="0")
        result_label.config(text="")
        more_button.grid_remove()

    def reset_all():
        for item in quantities:
            quantities[item] = 0
            qty_labels[item].config(text="0")
        result_label.config(text="")
        more_button.grid_remove()

    for idx, name in enumerate(["김밥", "삼각김밥", "라면"]):
        create_menu_item(name, idx)

    order_button = tk.Button(root, text="주문하기", command=place_order, bg="green", fg="white", font=("Arial", 20), width=15, height=2)
    order_button.grid(row=5, column=0, columnspan=3, pady=20)

    global result_label
    result_label = tk.Label(root, text="", font=("Arial", 20))
    result_label.grid(row=5, column=3, columnspan=3)

    global more_clicked, more_button
    more_clicked = [False]
    more_button = tk.Button(root, text="더 주문하시겠습니까?", command=order_more, bg="orange", font=("Arial", 18), width=25, height=2)
    more_button.grid(row=6, column=0, columnspan=6, pady=10)
    more_button.grid_remove()

    root.mainloop()
    ros_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
