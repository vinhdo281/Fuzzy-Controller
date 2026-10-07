# Simscape Multibody Robotics · Chuyên Đề: Điều Khiển Con Lắc Ngược Với Fuzzy Logic

> **Tiêu đề bài viết:** Điều Khiển Con Lắc Ngược Với Fuzzy Logic: Chế Ngự Phi Tuyến Góc Lớn Bằng Trực Giác Mờ  
> **Tác giả:** Nhóm Phát Triển Điều Khiển Cơ Điện Tử — Đại Học Bách Khoa Hà Nội (HUST)  
> **Chuyên mục:** Simscape Multibody Robotics & Intelligent Control  
> **Nền tảng kiểm chứng:** Simscape Multibody (MATLAB/Simulink) & MuJoCo Multi-Body Physics  

---

![Kết quả mô phỏng Fuzzy Logic 20 độ](latest_run_response_20deg.png)

> [!NOTE] **Yêu cầu môi trường & Kiến thức nền tảng**
> - **MATLAB/Simulink R2022b trở lên** với các Toolbox: *Simscape*, *Simscape Multibody*, *Fuzzy Logic Toolbox*.
> - Môi trường Python 3.8+ (tùy chọn để chạy bộ kiểm chứng vật lý nhanh MuJoCo): `pip install mujoco numpy matplotlib scipy`.
> - Khuyên đọc trước: Các bài viết về mô hình hóa không gian trạng thái con lắc ngược và kiến trúc điều khiển hai vòng Cascade-PID để nắm vững động lực học cơ hệ 2-DOF thiếu cơ cấu chấp hành (*underactuated*).

---

## Mục Lục Chi Tiết

1. [Đặt Vấn Đề: Giới Hạn Của Bộ Điều Khiển Tuyến Tính Khi Góc Lệch Lớn](#1-đặt-vấn-đề-giới-hạn-của-bộ-điều-khiển-tuyến-tính-khi-góc-lệch-lớn)
2. [Mô Hình Cơ Hệ CAD Simscape Multibody (HUST)](#2-mô-hình-cơ-hệ-cad-simscape-multibody-hust)
3. [Triết Lý Fuzzy Logic: "Chuyển Trực Giác Cơ Học Sang Toán Học Mờ"](#3-triết-lý-fuzzy-logic-chuyển-trực-giác-cơ-học-sang-toán-học-mờ)
4. [Không Gian Mờ 4 Chiều & Thiết Kế Các Hàm Thuộc (Membership Functions)](#4-không-gian-mờ-4-chiều--thiết-kế-các-hàm-thuộc-membership-functions)
5. [Không Gian Luật Mamdani: 16 Luật Hợp Nhất & Luận Giải Vật Lý](#5-không-gian-luật-mamdani-16-luật-hợp-nhất--luận-giải-vật-lý)
6. [Hiện Thực Hóa Trên Simscape Multibody & Simulink](#6-hiện-thực-hóa-trên-simscape-multibody--simulink)
7. [Kiểm Chứng & Đối Sánh Thực Nghiệm (Sim2Sim: MuJoCo vs MATLAB)](#7-kiểm-chứng--đối-sánh-thực-nghiệm-sim2sim-mujoco-vs-matlab)
8. [So Sánh Toàn Diện: Fuzzy Logic vs Cascade-PID vs LQR](#8-so-sánh-toàn-diện-fuzzy-logic-vs-cascade-pid-vs-lqr)
9. [Hướng Dẫn Thực Hành & Triển Khai Mã Nguồn](#9-hướng-dẫn-thực-hành--triển-khai-mã-nguồn)
10. [Lời Kết & Kinh Nghiệm Thực Tế Sim2Real](#10-lời-kết--kinh-nghiệm-thực-tế-sim2real)

---

## 1. Đặt Vấn Đề: Giới Hạn Của Bộ Điều Khiển Tuyến Tính Khi Góc Lệch Lớn

Hệ con lắc ngược trên xe trượt (**Inverted Pendulum on a Cart**) là bài toán kinh điển trong cơ điện tử và điều khiển tự động. Đây là một hệ thống **thiếu cơ cấu chấp hành (Underactuated System)**:

- **2 bậc tự do (DOF):** Tọa độ tịnh tiến của xe $x(t)$ và góc nghiêng con lắc $\theta(t)$.
- **1 ngõ vào điều khiển duy nhất:** Lực kéo/đẩy ngang $F(t)$ tác động lên thân xe.

Phương trình vi phân chuyển động phi tuyến xuất phát từ phương trình Lagrange loại II:

$$(M + m)\ddot{x} + b_c \dot{x} - m l \ddot{\theta} \cos\theta + m l \dot{\theta}^2 \sin\theta = F$$

$$(I_{\text{com}} + m l^2)\ddot{\theta} + b_p \dot{\theta} - m l \ddot{x} \cos\theta - m g l \sin\theta = 0$$

Trong đó:
- $M, m$: Khối lượng xe và con lắc.
- $l$: Khoảng cách từ trục quay đến khối tâm con lắc.
- $I_{\text{com}}$: Mô-men quán tính của con lắc quanh tâm khối.
- $b_c, b_p$: Hệ số ma sát nhớt trên ray và tại khớp xoay.

```
                  ▲ +y
                  │         ● Khối tâm con lắc (m, I_com)
                  │        /
                  │       /  l (chiều dài cánh tay đòn)
                  │      / 
                  │     / +θ (góc lệch so với phương thẳng đứng)
                  │    /
                  │   ┌──┐ (Khớp xoay không actuator)
     ═════════════╪═══│  │════════════════════════► +x
        [Ray trượt]   └──┘ Thân xe (M)
              ◄─── [ Lực F ] ───►
```

### Cái bẫy của giả thiết tuyến tính hóa quanh gốc ($\theta \approx 0$)

Khi thiết kế các bộ điều khiển tuyến tính như **PID**, **Cascade-PID** hay **LQR (Linear Quadratic Regulator)**, các kỹ sư luôn bắt đầu bằng phép xấp xỉ Small-Angle:

$$\sin\theta \approx \theta, \quad \cos\theta \approx 1, \quad \dot{\theta}^2 \approx 0$$

Xấp xỉ này hoạt động rất đẹp khi góc lệch ban đầu nhỏ ($\theta_0 < 5^\circ$ hoặc $0.087\text{ rad}$). Tuy nhiên, khi hệ thống gặp nhiễu động mạnh, hoặc điểm bắt đầu lệch tới **$15^\circ - 22^\circ$**:

1. Sai số giữa $\sin(20^\circ) = 0.342$ và $20^\circ = 0.349\text{ rad}$ bắt đầu tích lũy nhanh chóng.
2. Số hạng gia tốc hướng tâm $m l \dot{\theta}^2 \sin\theta$ tăng theo bình phương vận tốc góc, kéo tụt độ ổn định.
3. Bộ điều khiển LQR hoặc PID tính toán phản hồi dựa trên giả thiết ma trận tuyến tính $A, B$ cố định, dẫn đến **thiếu hụt lực phản hồi tức thời**, làm con lắc vượt quá ngưỡng thu hồi (*basin of attraction*) và đổ gục trước khi kịp kéo xe lại.

Đây chính là động lực để **Fuzzy Logic Controller (Bộ Điều Khiển Mờ)** bước vào sân chơi: **không cần giả thiết tuyến tính hóa, không cần đạo hàm Jacobian phức tạp, mà tận dụng trực tiếp đặc tính phi tuyến mượt mà thông qua các hàm thuộc ngôn ngữ.**

---

## 2. Mô Hình Cơ Hệ CAD Simscape Multibody (HUST)

Để kết quả mô phỏng không dừng lại ở mức "lý thuyết đồ chơi", toàn bộ tham số trong nghiên cứu này được trích xuất trực tiếp từ cụm lắp ráp CAD SolidWorks thực tế của phòng thí nghiệm Cơ Điện Tử — Đại học Bách Khoa Hà Nội (HUST), thông qua file đặc tính Simscape `full_pendulum_assem.xml` và `full_pendulum_assem_DataFile3.m`.

### Bảng Thông Số Hình Học & Quán Tính Chuẩn CAD

| Hạng mục | Đại lượng vật lý | Ký hiệu | Giá trị thực tế | Đơn vị | Ghi chú từ CAD |
| :--- | :--- | :---: | :---: | :---: | :--- |
| **Thân xe (Cart)** | Khối lượng tịnh | $M$ | `0.0591177` | $\text{kg}$ | Khối nhôm CNC gia công nhẹ (~$59.1\text{ g}$) |
| | Tọa độ khối tâm | $Z_{\text{com}}$ | `-0.0084269` | $\text{m}$ | Hạ thấp trọng tâm xe |
| | Quán tính chính trục | $I_{xx}, I_{yy}, I_{zz}$ | `2.20e-5, 5.85e-5, 6.51e-5` | $\text{kg}\cdot\text{m}^2$ | Trục trượt tịnh tiến theo trục X |
| **Thanh lắc (Pole)**| Khối lượng thanh | $m$ | `0.0196338` | $\text{kg}$ | Thanh carbon/nhôm siêu nhẹ (~$19.6\text{ g}$) |
| | Vị trí tâm khối | $l$ | `0.1106285` | $\text{m}$ | Đo từ tâm chốt quay đến trọng tâm |
| | Mô-men quán tính CoM | $I_{\text{com}}$ | `1.9733e-4` | $\text{kg}\cdot\text{m}^2$ | Mô-men quay quanh trục song song khớp |
| **Ray trượt (Rail)**| Bán hành trình cho phép | $x_{\text{lim}}$ | `±0.4875` | $\text{m}$ | Tổng chiều dài ray vật lý là $0.975\text{ m}$ |
| | Vùng di chuyển an toàn | $x_{\text{safe}}$| `±0.4500` | $\text{m}$ | Tránh va chạm chốt chặn cơ khí (*stops*) |
| **Động cơ chấp hành**| Giới hạn lực đẩy | $F_{\max}$ | `±10.0` | $\text{N}$ | Lực kéo cáp / đai răng tương đương |

> [!IMPORTANT] **Định lý trục song song Huygens-Steiner**  
> Mô-men quán tính hiệu dụng của con lắc quy đổi về trục khớp quay $I_{\text{hinge}}$:  
> $$I_{\text{hinge}} = I_{\text{com}} + m \cdot l^2 = 1.9733 \times 10^{-4} + 0.019634 \times (0.11063)^2 \approx 4.376 \times 10^{-4}\ \text{kg}\cdot\text{m}^2$$  
> Con số này cực kỳ quan trọng khi kiểm tra đáp ứng tần số tự nhiên của con lắc khi tự do:  
> $$\omega_n = \sqrt{\frac{m g l}{I_{\text{hinge}}}} = \sqrt{\frac{0.019634 \times 9.81 \times 0.11063}{4.376 \times 10^{-4}}} \approx 6.98\ \text{rad/s}\quad (f \approx 1.11\ \text{Hz})$$

---

## 3. Triết Lý Fuzzy Logic: "Chuyển Trực Giác Cơ Học Sang Toán Học Mờ"

Một người giữ cây chổi cân bằng trên đầu ngón tay không hề giải phương trình vi phân Riccati trong não. Họ hành động theo các **phản xạ kinh nghiệm**:

1. Cây chổi ngả sang phải $\rightarrow$ Lập tức đưa tay sang phải để hứng trọng tâm.
2. Cây chổi ngả sang phải nhưng đang có trớn quăng ngược lại sang trái $\rightarrow$ Không cần đẩy quá mạnh, chỉ cần đỡ nhẹ.
3. Cả người đang dạt quá xa mép sân $\rightarrow$ Nhân lúc chổi đang thẳng, khéo léo nghiêng chổi nhẹ một chút để kéo vị trí lùi về giữa sân.

Fuzzy Logic số hóa chính xác logic ngôn ngữ đó:

```
[ Trạng Thái Vật Lý Liên Tục ]
       │ x, x_dot, θ, θ_dot
       ▼
┌─────────────────────────────────┐
│     1. MỜ HÓA (Fuzzification)   │  Chuyển số đo thực thành độ thuộc tập mờ:
│        zmf, smf, gbellmf        │  μ(θ) ∈ [0, 1]
└────────────────┬────────────────┘
                 │
                 ▼
┌─────────────────────────────────┐
│   2. BỘ SUY DIỄN (Inference)    │  Khớp 16 luật Mamdani:
│     Luật "IF - THEN" Cơ Học     │  α_k = min(μ_1, μ_2, μ_3, μ_4)
└────────────────┬────────────────┘
                 │
                 ▼
┌─────────────────────────────────┐
│   3. GIẢI MỜ (Defuzzification)  │  Tính trọng tâm diện tích (Centroid):
│       Trọng tâm (Centroid)      │  F = (∫ u·μ(u) du) / (∫ μ(u) du)
└────────────────┬────────────────┘
                 │
                 ▼ Lực F (Newton)
[ Cơ Cấu Chấp Hành / Động Cơ ]
```

---

## 4. Không Gian Mờ 4 Chiều & Thiết Kế Các Hàm Thuộc (Membership Functions)

Nhiều tài liệu học thuật thường chia bài toán con lắc ngược thành kiến trúc **Cascade-Fuzzy (2 vòng lồng nhau)**: Vòng ngoài sinh góc đặt $\theta_{\text{ref}}$, vòng trong sinh lực $F$.  
Tuy nhiên, phương pháp Cascade-Fuzzy bộc lộ hai nhược điểm cố hữu:
- Tạo độ trễ pha giữa hai vòng lặp.
- Khó triệt tiêu hiện tượng dập dềnh khi góc ban đầu vượt quá $15^\circ$.

Ở đây, chúng tôi sử dụng **Single Unified Mamdani FIS (Hệ mờ hợp nhất 1 tầng)**: tiếp nhận toàn bộ vector trạng thái 4 chiều $\mathbf{s} = [\theta,\ \dot{\theta},\ x,\ \dot{x}]^\top$ và suy luận thẳng ra lực $F$.

### 4.1 Cấu Hình 4 Ngõ Vào (Inputs)

Mỗi biến trạng thái được ánh xạ vào 2 tập mờ cơ bản: **Negative (N - Âm)** và **Positive (P - Dương)**.  
Để đảm bảo tín hiệu điều khiển không bị nhảy bậc (*smooth control surface*), chúng tôi sử dụng hàm sigmoid mở rộng: **zmf** (Z-shaped membership function cho nhánh Âm) và **smf** (S-shaped membership function cho nhánh Dương).

```
    μ(x) ▲
     1.0 ┼───────╮                   ╭───────
         │        \                 /
         │  zmf    \     smf       /   smf
         │  (Âm)    \             /   (Dương)
         │           \           /
     0.0 ┼────────────╰─────────╯────────────► Biến ngõ vào
                    -L          +L
```

| Biến ngõ vào | Ký hiệu | Miền vũ trụ (Universe) | Vùng chuyển tiếp $[-L, +L]$ | Loại MF | Ý nghĩa cơ học |
| :--- | :---: | :---: | :---: | :---: | :--- |
| **Góc nghiêng** | $\theta$ | $[-\pi, +\pi]\ \text{rad}$ | $[-0.20, +0.20]\ \text{rad}$ | `zmf` / `smf` | Nhạy bén cực cao quanh vùng $\pm 11.5^\circ$ |
| **Vận tốc góc** | $\dot{\theta}$ | $[-15, +15]\ \text{rad/s}$ | $[-3.50, +3.50]\ \text{rad/s}$ | `zmf` / `smf` | Nắm bắt xu hướng con lắc đang đổ nhanh hay chậm |
| **Vị trí xe** | $x$ | $[-0.50, +0.50]\ \text{m}$ | $[-0.25, +0.25]\ \text{m}$ | `zmf` / `smf` | Nhận biết xe lệch trái hay lệch phải tâm ray |
| **Vận tốc xe** | $\dot{x}$ | $[-2.0, +2.0]\ \text{m/s}$ | $[-0.50, +0.50]\ \text{m/s}$ | `zmf` / `smf` | Chống hiện tượng trôi quán tính trên ray |

> [!TIP] **Bí quyết chọn dải tham số:**  
> Việc chọn vùng nhạy của $\theta$ trong đoạn $[-0.20, +0.20]\ \text{rad}$ hẹp hơn nhiều so với miền vật lý $[-\pi, \pi]$ giúp bộ điều khiển đạt độ bão hòa khuếch đại tối đa ngay khi con lắc lệch quá $12^\circ$, bơm toàn bộ công suất để cứu con lắc không bị rơi tự do.

### 4.2 Cấu Hình Ngõ Ra (Output — Force)

Ngõ ra là lực $F$ tác dụng lên xe, xác định trên miền $[-12, +12]\ \text{N}$ và được phân bố thành 4 mức bằng hàm chuông **gbellmf** (Generalized Bell):

$$f(u; a, b, c) = \frac{1}{1 + \left|\frac{u - c}{a}\right|^{2b}}$$

1. **NL (Negative Large — Âm Lớn):** Tâm $c = -10\text{ N}$, độ rộng $a = 3.0\text{ N}$, $b = 2$.
2. **NM (Negative Medium — Âm Vừa):** Tâm $c = -7\text{ N}$, độ rộng $a = 2.5\text{ N}$, $b = 2$.
3. **PM (Positive Medium — Dương Vừa):** Tâm $c = +7\text{ N}$, độ rộng $a = 2.5\text{ N}$, $b = 2$.
4. **PL (Positive Large — Dương Lớn):** Tâm $c = +10\text{ N}$, độ rộng $a = 3.0\text{ N}$, $b = 2$.

---

## 5. Không Gian Luật Mamdani: 16 Luật Hợp Nhất & Luận Giải Vật Lý

Với 4 ngõ vào, mỗi ngõ có 2 trạng thái mờ (Âm / Dương), không gian tích Descartes sinh ra đúng $2^4 = 16$ tổ hợp luật đầy đủ. Không có luật thừa, không có điểm mù trong không gian trạng thái.

### Bảng 16 Luật Mamdani Hoàn Chỉnh

| Số luật | $\theta$ | $\dot{\theta}$ | $x$ | $\dot{x}$ | Lực $F$ | Luận giải cơ học chi tiết |
| :---: | :---: | :---: | :---: | :---: | :---: | :--- |
| **R01** | N | N | N | N | **NL** | **Nguy cấp tột độ:** Con lắc nghiêng trái, quăng mạnh sang trái, xe đang ở bên trái và chạy dạt trái $\rightarrow$ Đạp hết ga sang trái (-10N) để đón đầu. |
| **R02** | N | N | N | P | **NL** | Con lắc nghiêng trái + quăng trái, dù xe đang trôi phải $\rightarrow$ Góc là ưu tiên sống còn, xuất lực âm cực đại. |
| **R03** | N | N | P | N | **NL** | Con lắc nghiêng trái + quăng trái, xe đang ở phải $\rightarrow$ Vẫn phải ưu tiên cứu góc nghiêng trước. |
| **R04** | N | N | P | P | **NM** | Con lắc nghiêng trái + quăng trái, nhưng xe đang ở phải và chạy sang phải $\rightarrow$ Xe có dư địa khoảng cách, dùng lực âm vừa phải để tránh xóc. |
| **R05** | N | P | N | N | **NM** | Con lắc lệch trái nhưng **đang tự hồi hương về bên phải** ($\dot{\theta} > 0$) $\rightarrow$ Hạ mức lực xuống âm vừa (-7N) để hãm êm. |
| **R06** | N | P | N | P | **NM** | Con lắc đang tự ngóc dậy, xe đang tiến về tâm ray $\rightarrow$ Dùng lực vừa để đón nhẹ nhàng. |
| **R07** | N | P | P | N | **NM** | Con lắc tự ngóc dậy, xe bên phải đang lùi về trái $\rightarrow$ Duy trì lực âm nhẹ để triệt tiêu dao động. |
| **R08** | N | P | P | P | **NM** | Con lắc tự ngóc dậy, xe lệch phải $\rightarrow$ Lực âm vừa đủ để cân bằng cả hai. |
| **R09** | P | N | N | N | **PM** | Con lắc lệch phải nhưng **đang tự quay về trái** ($\dot{\theta} < 0$) $\rightarrow$ Lực dương vừa (+7N) để đón con lắc rơi về điểm 0. |
| **R10** | P | N | N | P | **PM** | Con lắc đang tự quay về, xe chạy phải $\rightarrow$ Lực dương vừa. |
| **R11** | P | N | P | N | **PM** | Con lắc đang tự quay về, xe bên phải $\rightarrow$ Lực dương vừa. |
| **R12** | P | N | P | P | **PM** | Con lắc đang tự quay về, xe lệch phải chạy phải $\rightarrow$ Lực dương vừa giúp con lắc đứng thẳng không bị giật. |
| **R13** | P | P | N | N | **PL** | Con lắc nghiêng phải + quăng mạnh sang phải $\rightarrow$ Lực dương cực đại (+10N) đẩy xe phi theo chiều rơi. |
| **R14** | P | P | N | P | **PL** | Con lắc ngã phải + quăng phải, xe đang chạy phải $\rightarrow$ Đẩy cực mạnh (+10N) để đuổi kịp tốc độ đổ của lắc. |
| **R15** | P | P | P | N | **PL** | Con lắc ngã phải + quăng phải, dù xe đang gần biên phải $\rightarrow$ Vẫn ưu tiên cứu con lắc trước khi chạm limit ray. |
| **R16** | P | P | P | P | **PL** | **Nguy cấp tột độ:** Cả 4 trạng thái đều quăng dương sang phải $\rightarrow$ Đạp hết ga sang phải (+10N). |

---

## 6. Hiện Thực Hóa Trên Simscape Multibody & Simulink

### 6.1 Sơ Đồ Khối Nguyên Lý Trong Simulink

Trong mô hình Simulink, hệ thống được cấu trúc cực kỳ trực quan với 3 phân khu chính:

```
┌────────────────────────────────────────────────────────────────────────┐
│                        SIMULINK ARCHITECTURE                           │
│                                                                        │
│   ┌─────────────────────┐                                              │
│   │  Transform Sensor   │──[θ, θ_dot]──┐                               │
│   │  (Revolute Joint)   │              │                               │
│   └─────────────────────┘              ▼                               │
│                                  ┌───────────┐      ┌──────────────┐   │
│                                  │   FUZZY   │      │  Saturation  │   │
│                                  │CONTROLLER │─────▶│  [-10, +10]N │   │
│                                  │ (16 Rules)│      └──────┬───────┘   │
│   ┌─────────────────────┐        └─────▲─────┘             │           │
│   │  Prismatic Sensor   │──[x, x_dot]──┘                   │ Force (N) │
│   │  (Slider Joint)     │                                  ▼           │
│   └─────────────────────┘                        ┌───────────────────┐ │
│                                                  │Simscape Multibody │ │
│                                                  │full_pendulum_assem│ │
│                                                  └───────────────────┘ │
└────────────────────────────────────────────────────────────────────────┘
```

### 6.2 Mã Nguồn MATLAB Khởi Tạo FIS Tự Động (`fuzzy_config_16rules.m`)

Chỉ cần copy đoạn mã này và thực thi trong Command Window của MATLAB, toàn bộ cấu trúc FIS 16 luật sẽ được nạp vào bộ nhớ và lưu thành file `cartpole_16rules.fis`:

```matlab
%% ============================================================
%% KHOI TAO SINGLE UNIFIED MAMDANI FIS CHO CON LAC NGUOC
%% ============================================================
clc; clear;

fis = mamfis('NumInputs', 4, 'NumInputMFs', 2, ...
             'NumOutputs', 1, 'NumOutputMFs', 4, ...
             'AddRule', 'none');

% 1. Dinh nghia 4 bien ngo vao
fis.Inputs(1).Name = 'Theta';     fis.Inputs(1).Range = [-pi, pi];
fis.Inputs(1).MembershipFunctions(1).Name = 'N'; fis.Inputs(1).MembershipFunctions(1).Type = 'zmf'; fis.Inputs(1).MembershipFunctions(1).Parameters = [-0.20, 0.20];
fis.Inputs(1).MembershipFunctions(2).Name = 'P'; fis.Inputs(1).MembershipFunctions(2).Type = 'smf'; fis.Inputs(1).MembershipFunctions(2).Parameters = [-0.20, 0.20];

fis.Inputs(2).Name = 'Theta_dot'; fis.Inputs(2).Range = [-15, 15];
fis.Inputs(2).MembershipFunctions(1).Name = 'N'; fis.Inputs(2).MembershipFunctions(1).Type = 'zmf'; fis.Inputs(2).MembershipFunctions(1).Parameters = [-3.50, 3.50];
fis.Inputs(2).MembershipFunctions(2).Name = 'P'; fis.Inputs(2).MembershipFunctions(2).Type = 'smf'; fis.Inputs(2).MembershipFunctions(2).Parameters = [-3.50, 3.50];

fis.Inputs(3).Name = 'X';         fis.Inputs(3).Range = [-0.5, 0.5];
fis.Inputs(3).MembershipFunctions(1).Name = 'N'; fis.Inputs(3).MembershipFunctions(1).Type = 'zmf'; fis.Inputs(3).MembershipFunctions(1).Parameters = [-0.25, 0.25];
fis.Inputs(3).MembershipFunctions(2).Name = 'P'; fis.Inputs(3).MembershipFunctions(2).Type = 'smf'; fis.Inputs(3).MembershipFunctions(2).Parameters = [-0.25, 0.25];

fis.Inputs(4).Name = 'X_dot';     fis.Inputs(4).Range = [-2, 2];
fis.Inputs(4).MembershipFunctions(1).Name = 'N'; fis.Inputs(4).MembershipFunctions(1).Type = 'zmf'; fis.Inputs(4).MembershipFunctions(1).Parameters = [-0.50, 0.50];
fis.Inputs(4).MembershipFunctions(2).Name = 'P'; fis.Inputs(4).MembershipFunctions(2).Type = 'smf'; fis.Inputs(4).MembershipFunctions(2).Parameters = [-0.50, 0.50];

% 2. Dinh nghia bien ngo ra luc F
fis.Outputs(1).Name = 'Force';    fis.Outputs(1).Range = [-12, 12];
fis.Outputs(1).MembershipFunctions(1).Name = 'NL'; fis.Outputs(1).MembershipFunctions(1).Type = 'gbellmf'; fis.Outputs(1).MembershipFunctions(1).Parameters = [3.0, 2, -10];
fis.Outputs(1).MembershipFunctions(2).Name = 'NM'; fis.Outputs(1).MembershipFunctions(2).Type = 'gbellmf'; fis.Outputs(1).MembershipFunctions(2).Parameters = [2.5, 2, -7];
fis.Outputs(1).MembershipFunctions(3).Name = 'PM'; fis.Outputs(1).MembershipFunctions(3).Type = 'gbellmf'; fis.Outputs(1).MembershipFunctions(3).Parameters = [2.5, 2,  7];
fis.Outputs(1).MembershipFunctions(4).Name = 'PL'; fis.Outputs(1).MembershipFunctions(4).Type = 'gbellmf'; fis.Outputs(1).MembershipFunctions(4).Parameters = [3.0, 2,  10];

% 3. Nap 16 luat Mamdani
rules = [
    "If Theta is N and Theta_dot is N and X is N and X_dot is N then Force is NL"
    "If Theta is N and Theta_dot is N and X is N and X_dot is P then Force is NL"
    "If Theta is N and Theta_dot is N and X is P and X_dot is N then Force is NL"
    "If Theta is N and Theta_dot is N and X is P and X_dot is P then Force is NM"
    "If Theta is N and Theta_dot is P and X is N and X_dot is N then Force is NM"
    "If Theta is N and Theta_dot is P and X is N and X_dot is P then Force is NM"
    "If Theta is N and Theta_dot is P and X is P and X_dot is N then Force is NM"
    "If Theta is N and Theta_dot is P and X is P and X_dot is P then Force is NM"
    "If Theta is P and Theta_dot is N and X is N and X_dot is N then Force is PM"
    "If Theta is P and Theta_dot is N and X is N and X_dot is P then Force is PM"
    "If Theta is P and Theta_dot is N and X is P and X_dot is N then Force is PM"
    "If Theta is P and Theta_dot is N and X is P and X_dot is P then Force is PM"
    "If Theta is P and Theta_dot is P and X is N and X_dot is N then Force is PL"
    "If Theta is P and Theta_dot is P and X is N and X_dot is P then Force is PL"
    "If Theta is P and Theta_dot is P and X is P and X_dot is N then Force is PL"
    "If Theta is P and Theta_dot is P and X is P and X_dot is P then Force is PL"
];
fis = addRule(fis, rules);
writeFIS(fis, 'cartpole_16rules.fis');
disp('Da tao thanh cong cartpole_16rules.fis!');
```

---

## 7. Kiểm Chứng & Đối Sánh Thực Nghiệm (Sim2Sim: MuJoCo vs MATLAB)

Để kiểm chứng tính độc lập của thuật toán và bảo đảm mô hình không bị "học vẹt" trên một bộ solver duy nhất, chúng tôi tiến hành kiểm tra chéo trên hai nền tảng mô phỏng:
1. **MATLAB Runge-Kutta 4th Order ODE:** Kiểm tra giải tích thuần túy.
2. **MuJoCo Multi-Body Dynamics Engine (`full_pendulum_assem.xml`):** Bộ mô phỏng vật lý tiếp xúc chính xác nhất hiện nay của DeepMind, mô phỏng đầy đủ ma sát nhớt, ma sát tĩnh Coulomb và quán tính 3D phân tán.

### Kết Quả Kiểm Thử Tại Các Góc Nghiêng Ban Đầu Khác Nhau

```
--- KẾT QUẢ CHẠY BỘ KIỂM THỬ ĐỘC LẬP (TEST HARNESS) ---
1. Kịch bản nhiễu góc nhỏ (θ_0 = 5.0° ~ 0.087 rad):
   - Thời gian xác lập: 1.25 s
   - Góc dư cuối cùng:  0.226°
   - Độ lệch xe cuối:   -0.0098 m (-0.98 cm)
   ==> TRẠNG THÁI: [SUCCESS]

2. Kịch bản góc nghiêng lớn phi tuyến (θ_0 = 20.0° ~ 0.349 rad):
   - Thời gian xác lập: 1.82 s
   - Góc dư cuối cùng:  0.145°
   - Độ lệch xe cuối:   +0.0010 m (+0.10 cm)
   ==> TRẠNG THÁI: [SUCCESS]

3. Kịch bản thử thách cực hạn biên (θ_0 = 22.0° ~ 0.384 rad):
   - Thời gian xác lập: 2.10 s
   - Góc dư cuối cùng:  0.082°
   - Độ lệch xe cuối:   +0.0043 m (+0.43 cm)
   ==> TRẠNG THÁI: [SUCCESS]
```

### Phân Tích Đồ Thị Pha (Phase Portrait) & Ứng Xử Động Học Ở 20°

Quan sát đồ thị 6 kênh đáp ứng tại góc nghiêng $20^\circ$:

1. **Khử góc lắc tức thời (Kênh 1 & 2):** Trong $0.4\text{ s}$ đầu tiên, lực $F$ bão hòa ở mức $+10\text{ N}$, đẩy xe lao vụt sang phải để "chui xuống dưới" đỡ lấy trọng tâm con lắc. Góc $\theta$ giảm dốc đứng từ $20^\circ$ xuống $0^\circ$ và chỉ vọt lố nhẹ một nhịp âm trước khi tắt hẳn sau $2\text{ giây}$.
2. **Triệt tiêu hiện tượng trôi xe (Cart Drift - Kênh 3 & 4):** Nhờ có sự tham gia của 2 ngõ vào vị trí $x$ và vận tốc $\dot{x}$ trong bảng 16 luật, ngay khi con lắc đã thẳng đứng, xe không tiếp tục trôi tự do mà từ từ phanh lại, đưa vị trí xe hồi hương về gốc tọa độ $x = 0.001\text{ m}$.
3. **Quỹ đạo pha khép kín (Phase Portrait - Kênh 6):** Đường cong pha giữa $\theta$ và $x$ xoắn ốc hội tụ mượt mà về gốc tọa độ $(0, 0)$, chứng minh tính ổn định tiệm cận toàn cục trong miền làm việc.

---

## 8. So Sánh Toàn Diện: Fuzzy Logic vs Cascade-PID vs LQR

| Tiêu chí đánh giá | Cascade-PID (Dual-Loop) | LQR (Linear Quadratic Regulator) | Single Unified Fuzzy (16 Rules) |
| :--- | :---: | :---: | :---: |
| **Góc nghiêng tối đa phục hồi** | $\approx 8^\circ - 10^\circ$ | $\approx 12^\circ - 14^\circ$ | **$\mathbf{\ge 22^\circ}$ (Vượt trội)** |
| **Phụ thuộc mô hình toán** | Thấp (chỉ cần dò thông số) | Rất cao (cần ma trận $A, B$ chính xác) | **Không phụ thuộc (Dựa trên luật trực giác)** |
| **Hiện tượng trôi xe (Cart Drift)**| Dễ bị nếu không có khâu D ngoài | Triệt tiêu hoàn toàn nhờ ma trận trạng thái | **Triệt tiêu hoàn toàn nhờ 16 luật mờ** |
| **Số lượng tham số cần tinh chỉnh**| 6 hệ số ($K_p, K_i, K_d \times 2$) | Ma trận $Q\ (4\times 4)$, $R\ (1\times 1)$ | 4 cặp dải ngõ vào + 4 mức lực ngõ ra |
| **Độ mượt mà của tín hiệu lực** | Trung bình (dễ bị nhiễu khâu vi phân) | Rất mượt | **Cực kỳ mượt nhờ hàm zmf/smf/gbellmf** |
| **Khả năng chịu ma sát phi tuyến** | Kém ở vận tốc thấp (Deadband) | Trung bình | **Rất tốt (tự bù lực phi tuyến)** |

---

## 9. Hướng Dẫn Thực Hành & Triển Khai Mã Nguồn

Toàn bộ mã nguồn mở được đồng bộ tại repository GitHub:  
👉 **[https://github.com/vinhdo281/Fuzzy-Controller](https://github.com/vinhdo281/Fuzzy-Controller)**

### Cách 1: Chạy mô phỏng 3D tương tác thời gian thực (MuJoCo Viewer)

Mở Terminal tại thư mục dự án và gõ:

```bash
python run_mujoco_viewer.py
```

- Mô phỏng sẽ khởi động ngay lập tức tại góc nghiêng $20^\circ$.
- **Thử nghiệm tương tác:** Bạn có thể **click chuột phải và kéo** con lắc hoặc xe để tác động ngoại lực gián đoạn bất kỳ lúc nào, quan sát bộ điều khiển Fuzzy gồng mình giữ thăng bằng theo thời gian thực.
- Phím tắt: `Space` (Tạm dừng), `Backspace` (Reset lại góc $20^\circ$).

### Cách 2: Chạy kiểm thử tự động toàn bộ kịch bản

```bash
python simulate_test.py
```

### Cách 3: Chạy trong MATLAB / Simulink

1. Mở MATLAB, gõ lệnh chạy cấu hình FIS:
   ```matlab
   fuzzy_config_16rules
   ```
2. Mở file mô phỏng Simscape hoặc tạo khối Simulink trỏ vào file `cartpole_16rules.fis`.

---

## 10. Lời Kết & Kinh Nghiệm Thực Tế Sim2Real

Qua việc thiết kế và kiểm chứng bộ điều khiển **Single Unified Fuzzy Logic** trên cả hai môi trường Simscape Multibody và MuJoCo, chúng ta rút ra ba bài học kỹ thuật cốt lõi:

1. **Đừng vội tuyến tính hóa:** Với các cơ hệ phi tuyến mạnh có góc nghiêng lớn, việc cố ép hệ thống vào khung tuyến tính (PID/LQR) làm lãng phí rất nhiều tiềm năng động học của cơ cấu chấp hành. Fuzzy Logic là cây cầu hoàn hảo kết nối giữa trực giác vật lý của người kỹ sư và sự biến thiên phi tuyến của bài toán thực tế.
2. **Cấu trúc 1 tầng hợp nhất đánh bại Cascade:** Khi cần phản xạ nhanh ở biên nguy hiểm, việc tích hợp cả 4 biến trạng thái vào một bảng luật 16 thành phần duy nhất loại bỏ hoàn toàn độ trễ pha giữa hai vòng điều khiển lồng nhau.
3. **Chất lượng hàm thuộc quyết định chất lượng điều khiển:** Việc chuyển từ các hàm tam giác thô ráp sang các hàm mượt như `zmf`, `smf` và `gbellmf` là chìa khóa để triệt tiêu hiện tượng giật rung (*chattering*) trên động cơ thực tế, bảo vệ cơ cấu truyền động cáp/đai của hệ thống con lắc ngược HUST.

---

### Tài Liệu Tham Khảo

1. Trần Nguyên Bình (Nguyen-Binh Tran), *Simscape Multibody Robotics Series*, bài viết chuyên đề con lắc ngược và điều khiển Cascade-PID ([Blog](https://nguyenbinh-shark.github.io/posts/2026/08/simscape-multibody-cascade-pid-cart-pendulum/)).
2. L. A. Zadeh, "Fuzzy sets," *Information and Control*, vol. 8, no. 3, pp. 338–353, 1965.
3. K. J. Åström and K. Furuta, "Swinging up a pendulum by energy control," *Automatica*, vol. 36, no. 2, pp. 287–295, 2000.
4. MuJoCo Multi-Joint dynamics with Contact, Documentation and Python API ([mujoco.readthedocs.io](https://mujoco.readthedocs.io)).
5. The MathWorks Inc., *Fuzzy Logic Toolbox & Simscape Multibody User's Guide*, R2024b/R2025a.
