import time

import streamlit as st
import streamlit.components.v1 as components

import warehouse_db
from streamlit_ros_client import get_ros_client

st.set_page_config(page_title="WAREHOUSE MANAGEMENT SYSTEM ", layout="centered")
warehouse_db.init_db()
ros_client = get_ros_client()

JOYSTICK_HTML = """
<!DOCTYPE html>
<html lang="en">
<head>
  <title>Robot Control</title>
  <meta name="viewport" content="width=device-width, initial-scale=1.0">
  <script src="https://cdn.jsdelivr.net/npm/nipplejs@0.10.1/dist/nipplejs.min.js"></script>
  <style>
    * { margin: 0; padding: 0; box-sizing: border-box; }
    body {
      background: #0d1117;
      min-height: 100vh;
      display: flex;
      flex-direction: column;
      align-items: center;
      justify-content: center;
      font-family: 'SF Mono', 'Fira Code', monospace;
      padding: 24px 16px;
    }
    .header { display: flex; align-items: center; gap: 10px; margin-bottom: 28px; }
    .header h1 { font-size: 15px; font-weight: 600; color: #e6edf3; letter-spacing: 0.08em; text-transform: uppercase; }
    .header p { font-size: 11px; color: #7d8590; letter-spacing: 0.05em; margin-top: 2px; }
    .conn-bar {
      display: flex; align-items: center; gap: 8px;
      background: #161b22; border: 0.5px solid #30363d; border-radius: 20px;
      padding: 6px 14px; margin-bottom: 28px; font-size: 12px; color: #7d8590;
      transition: all 0.3s;
    }
    .conn-bar.connected { border-color: #00d4aa44; color: #00d4aa; }
    .conn-dot { width: 7px; height: 7px; border-radius: 50%; background: #f85149; transition: background 0.3s; }
    .conn-bar.connected .conn-dot { background: #00d4aa; animation: pulse 2s infinite; }
    @keyframes pulse { 0%,100%{opacity:1} 50%{opacity:0.4} }
    .joystick-wrap { position: relative; margin-bottom: 32px; }
    .joystick-ring {
      width: 240px; height: 240px; border-radius: 50%;
      background: #161b22; border: 1.5px solid #30363d;
      position: relative; display: flex; align-items: center; justify-content: center;
    }
    .joystick-ring::before { content: ''; position: absolute; inset: 16px; border-radius: 50%; border: 0.5px dashed #21262d; }
    .cross-h, .cross-v { position: absolute; background: #21262d; }
    .cross-h { width: 80%; height: 0.5px; top: 50%; left: 10%; }
    .cross-v { height: 80%; width: 0.5px; left: 50%; top: 10%; }
    .dir-label { position: absolute; font-size: 9px; color: #484f58; letter-spacing: 0.1em; font-weight: 600; }
    .dir-label.n { top: 16px; left: 50%; transform: translateX(-50%); }
    .dir-label.s { bottom: 16px; left: 50%; transform: translateX(-50%); }
    .dir-label.w { left: 16px; top: 50%; transform: translateY(-50%); }
    .dir-label.e { right: 16px; top: 50%; transform: translateY(-50%); }
    #zone { position: absolute; inset: 0; border-radius: 50%; z-index: 10; }
    .metrics { display: grid; grid-template-columns: 1fr 1fr; gap: 10px; width: 100%; max-width: 320px; margin-bottom: 20px; }
    .metric-card { background: #161b22; border: 0.5px solid #30363d; border-radius: 10px; padding: 12px 14px; }
    .metric-label { font-size: 10px; color: #7d8590; letter-spacing: 0.1em; text-transform: uppercase; margin-bottom: 6px; }
    .metric-value { font-size: 20px; font-weight: 600; color: #e6edf3; }
    .metric-value span { font-size: 11px; color: #7d8590; font-weight: 400; margin-left: 3px; }
    .metric-bar { margin-top: 8px; height: 3px; background: #21262d; border-radius: 2px; overflow: hidden; }
    .metric-fill { height: 100%; border-radius: 2px; background: #00d4aa; width: 0%; transition: width 0.1s; }
    .metric-fill.angular { background: #58a6ff; }
    .footer { font-size: 10px; color: #484f58; letter-spacing: 0.06em; }
  </style>
</head>
<body>
  <div class="header">
    <div>
      <h1>Robot Control</h1>
      <p>joystick Mode</p>
    </div>
  </div>

  <div class="conn-bar" id="connBar">
    <div class="conn-dot"></div>
    <span id="connText">Connecting to ws://192.168.3.235:8765</span>
  </div>

  <div class="joystick-wrap">
    <div class="joystick-ring">
      <div class="cross-h"></div>
      <div class="cross-v"></div>
      <span class="dir-label n">LEFT</span>
      <span class="dir-label s">RIGHT</span>
      <span class="dir-label w">FWD</span>
      <span class="dir-label e">BWD</span>
      <div id="zone"></div>
    </div>
  </div>

  <div class="metrics">
    <div class="metric-card">
      <div class="metric-label">Linear</div>
      <div class="metric-value" id="linVal">0.00<span>m/s</span></div>
      <div class="metric-bar"><div class="metric-fill" id="linBar"></div></div>
    </div>
    <div class="metric-card">
      <div class="metric-label">Angular</div>
      <div class="metric-value" id="angVal">0.00<span>r/s</span></div>
      <div class="metric-bar"><div class="metric-fill angular" id="angBar"></div></div>
    </div>
  </div>

  <div class="footer">MAX VEL 0.84 m/s &middot; 1.5 rad/s &middot; 20 Hz</div>

  <script>
    const MAX_LINEAR  = 0.84;
    const MAX_ANGULAR = 1.5;

    const connBar  = document.getElementById('connBar');
    const connText = document.getElementById('connText');
    const linVal   = document.getElementById('linVal');
    const angVal   = document.getElementById('angVal');
    const linBar   = document.getElementById('linBar');
    const angBar   = document.getElementById('angBar');

    let ws;

    function connect() {
      ws = new WebSocket("ws://192.168.3.235:8765");

      ws.onopen = () => {
        connBar.className = 'conn-bar connected';
        connText.textContent = 'Connected · ws://192.168.3.235:8765';
      };

      ws.onclose = () => {
        connBar.className = 'conn-bar';
        connText.textContent = 'Disconnected — retrying...';
        setTimeout(connect, 2000);
      };
      ws.onerror = () => ws.close();
    }

    connect();

    const joystick = nipplejs.create({
      zone: document.getElementById('zone'),
      mode: 'static',
      position: { left: '50%', top: '50%' },
      color: '#00d4aa',
      size: 80
    });

    function updateMetrics(linear, angular) {
      linVal.innerHTML = Math.abs(linear).toFixed(2) + '<span>m/s</span>';
      angVal.innerHTML = Math.abs(angular).toFixed(2) + '<span>r/s</span>';
      linBar.style.width = (Math.abs(linear) / MAX_LINEAR * 100) + '%';
      angBar.style.width  = (Math.abs(angular) / MAX_ANGULAR * 100) + '%';
      linBar.style.background = linear < 0 ? '#f85149' : '#00d4aa';
    }

    joystick.on('move', (evt, data) => {
      const angle  = data.angle.radian;
      const force  = Math.min(data.force, 1.0);
      const linear  =  Math.sin(angle) * force * MAX_LINEAR;
      const angular = -Math.cos(angle) * force * MAX_ANGULAR;
      updateMetrics(linear, angular);
      if (ws && ws.readyState === WebSocket.OPEN) {
        ws.send(JSON.stringify({ linear, angular }));
      }
    });

    joystick.on('end', () => {
      updateMetrics(0, 0);
      if (ws && ws.readyState === WebSocket.OPEN) {
        ws.send(JSON.stringify({ linear: 0.0, angular: 0.0 }));
      }
    });
  </script>
</body>
</html>
"""


def wait_for_state(subsystem: str, target_state: str, timeout_s: float = 30.0) -> bool:
    """Poll ros_client state until it reaches target_state or an error/timeout occurs."""
    deadline = time.time() + timeout_s
    time.sleep(1.0)  # give the Arduino a moment to ack and go BUSY first
    while time.time() < deadline:
        if subsystem == 'lift':
            current_state, error = ros_client.lift_state, ros_client.last_lift_error
        else:
            current_state, error = ros_client.carriage_state, ros_client.last_carriage_error

        if error:
            return False
        if current_state == target_state:
            return True
        time.sleep(0.3)
    return False


def run_pick_sequence(order: dict) -> bool:
    """
    Lifts to the shelf height, then runs the carriage pick. /mission/busy is
    held True for the whole sequence so joystick_ws_node.py zeroes /cmd_vel
    and the base can't be driven while the arm is moving — this replaces the
    interlock mission_node.py used to provide.
    """
    target_height = order['height_mm']

    ros_client.set_mission_busy(True)
    try:
        with st.spinner(f"Lifting to {target_height:.0f} mm (Shelf {order['shelf_label']})..."):
            ros_client.send_lift_to_height(target_height)
            if not wait_for_state('lift', 'IDLE'):
                st.error(f"Lift did not finish in time. Last error: {ros_client.last_lift_error}")
                return False

        with st.spinner("Running pick sequence..."):
            ros_client.send_carriage_pick()
            if not wait_for_state('carriage', 'IDLE'):
                st.error(f"Carriage pick did not finish in time. Last error: {ros_client.last_carriage_error}")
                return False

        return True
    finally:
        # always release the interlock, whether the sequence succeeded, failed,
        # or timed out — the base must never be left permanently locked out
        ros_client.set_mission_busy(False)


st.title("Warehouse AMR Control")

tab_racks, tab_orders, tab_drive = st.tabs(["Racks", "Orders", "Drive & Pick"])

# ── Racks tab ────────────────────────────────────────────────────────
with tab_racks:
    st.subheader("Add a rack")
    st.caption("Every rack automatically gets 3 shelves")
    with st.form("add_rack_form", clear_on_submit=True):
        rack_name = st.text_input("Rack name (e.g. Rack A)")
        rack_notes = st.text_input("Notes (optional)")
        submitted = st.form_submit_button("Add rack")
        if submitted:
            if rack_name.strip():
                warehouse_db.add_rack(rack_name.strip(), rack_notes.strip())
                st.success(f"Added {rack_name} with shelves A/B/C")
            else:
                st.warning("Rack name can't be empty.")

    st.subheader("Existing racks")
    racks = warehouse_db.list_racks()
    if racks:
        st.dataframe(racks, use_container_width=True)
    else:
        st.info("No racks yet — add one above.")

    st.subheader("Shelves")
    shelves = warehouse_db.list_shelves()
    if shelves:
        st.dataframe(shelves, use_container_width=True)
    else:
        st.info("No shelves yet — add a rack above.")

# ── Orders tab ───────────────────────────────────────────────────────
with tab_orders:
    st.subheader("Create an order")
    racks = warehouse_db.list_racks()
    if not racks:
        st.info("Add a rack first, in the Racks tab.")
    else:
        rack_options = {rack['name']: rack['id'] for rack in racks}
        selected_rack_name = st.selectbox("Rack", list(rack_options.keys()))
        selected_rack_id = rack_options[selected_rack_name]

        shelves_for_rack = warehouse_db.list_shelves(selected_rack_id)
        shelf_options = {
            f"Shelf {shelf['label']} ({shelf['height_mm']:.0f} mm)": shelf['id']
            for shelf in shelves_for_rack
        }
        selected_shelf_label = st.selectbox("Shelf", list(shelf_options.keys()))

        if st.button("Create order"):
            new_order_id = warehouse_db.create_order(shelf_options[selected_shelf_label])
            st.success(f"Order #{new_order_id} created for {selected_rack_name} — {selected_shelf_label}")

    st.subheader("All orders")
    orders = warehouse_db.list_orders()
    if orders:
        st.dataframe(orders, use_container_width=True)
    else:
        st.info("No orders yet.")

# ── Drive & Pick tab ─────────────────────────────────────────────────
with tab_drive:
    st.subheader("Homing (do this once at startup)")
    col_home_lift, col_home_carriage = st.columns(2)
    with col_home_lift:
        if st.button("Home Lift"):
            ros_client.send_lift_home()
            st.info("Home command sent to lift.")
    with col_home_carriage:
        if st.button("Home Carriage"):
            ros_client.send_carriage_home()
            st.info("Home command sent to carriage.")

    st.caption(
        f"Lift: {ros_client.lift_state} · {ros_client.lift_height_mm:.0f} mm  |  "
        f"Carriage: {ros_client.carriage_state}"
    )

    st.divider()

    st.subheader("Pending orders")
    pending_orders = [order for order in warehouse_db.list_orders() if order['status'] == 'pending']

    if not pending_orders:
        st.info("No pending orders — create one in the Orders tab.")
    else:
        order_labels = {
            f"Order #{order['id']} — {order['rack_name']} / Shelf {order['shelf_label']}": order
            for order in pending_orders
        }
        selected_label = st.selectbox("Select order", list(order_labels.keys()))
        selected_order = order_labels[selected_label]

        st.write(f"Rack: **{selected_order['rack_name']}**  ·  "
                 f"Shelf: **{selected_order['shelf_label']}**  ·  "
                 f"Lift height: **{selected_order['height_mm']:.0f} mm**")

        if "show_joystick" not in st.session_state:
            st.session_state.show_joystick = False

        if st.button("Go to Joystick Mode"):
            st.session_state.show_joystick = True

        if st.session_state.show_joystick:
            st.caption("Drive the robot to the rack, then come back and click Start.")
            components.html(JOYSTICK_HTML, height=650, scrolling=False)

        st.divider()
        if st.button("Start Pick Sequence", type="primary"):
            warehouse_db.update_order_status(selected_order['id'], "in_progress")
            success = run_pick_sequence(selected_order)
            if success:
                warehouse_db.update_order_status(selected_order['id'], "completed")
                st.success(f"Order #{selected_order['id']} completed.")
                st.session_state.show_joystick = False
                st.rerun()
            else:
                warehouse_db.update_order_status(selected_order['id'], "error")