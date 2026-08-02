import sqlite3
from pathlib import Path
from datetime import datetime

DB_PATH = Path(__file__).parent / "warehouse.db"

# Every rack has exactly these 3 shelves, at these fixed heights.
SHELF_HEIGHTS_MM = {
    "A": 50.0,
    "B": 150.0,
    "C": 200.0,
}


def get_connection():
    connection = sqlite3.connect(DB_PATH, check_same_thread=False)
    connection.row_factory = sqlite3.Row
    connection.execute("PRAGMA foreign_keys = ON")
    return connection


def init_db():
    connection = get_connection()
    cursor = connection.cursor()
    cursor.execute("""
        CREATE TABLE IF NOT EXISTS racks (
            id INTEGER PRIMARY KEY AUTOINCREMENT,
            name TEXT UNIQUE NOT NULL,
            notes TEXT
        )
    """)
    cursor.execute("""
        CREATE TABLE IF NOT EXISTS shelves (
            id INTEGER PRIMARY KEY AUTOINCREMENT,
            rack_id INTEGER NOT NULL,
            label TEXT NOT NULL,
            height_mm REAL NOT NULL,
            UNIQUE (rack_id, label),
            FOREIGN KEY (rack_id) REFERENCES racks (id) ON DELETE CASCADE
        )
    """)
    cursor.execute("""
        CREATE TABLE IF NOT EXISTS orders (
            id INTEGER PRIMARY KEY AUTOINCREMENT,
            shelf_id INTEGER NOT NULL,
            status TEXT NOT NULL DEFAULT 'pending',
            created_at TEXT NOT NULL,
            completed_at TEXT,
            FOREIGN KEY (shelf_id) REFERENCES shelves (id) ON DELETE CASCADE
        )
    """)
    connection.commit()
    connection.close()


def add_rack(name, notes=""):
    """
    Create a rack and automatically attach its 3 fixed-height shelves
    (A -> 150mm, B -> 400mm, C -> 650mm). Returns the new rack id.
    """
    connection = get_connection()
    cursor = connection.cursor()
    cursor.execute(
        "INSERT INTO racks (name, notes) VALUES (?, ?)",
        (name, notes),
    )
    rack_id = cursor.lastrowid
    for label, height_mm in SHELF_HEIGHTS_MM.items():
        cursor.execute(
            "INSERT INTO shelves (rack_id, label, height_mm) VALUES (?, ?, ?)",
            (rack_id, label, height_mm),
        )
    connection.commit()
    connection.close()
    return rack_id


def list_racks():
    connection = get_connection()
    cursor = connection.cursor()
    cursor.execute("SELECT * FROM racks ORDER BY name")
    rows = cursor.fetchall()
    connection.close()
    return [dict(row) for row in rows]


def get_rack(rack_id):
    connection = get_connection()
    cursor = connection.cursor()
    cursor.execute("SELECT * FROM racks WHERE id = ?", (rack_id,))
    row = cursor.fetchone()
    connection.close()
    return dict(row) if row else None


def list_shelves(rack_id=None):
    """List shelves, optionally filtered to a single rack, joined with rack name."""
    connection = get_connection()
    cursor = connection.cursor()
    if rack_id is None:
        cursor.execute("""
            SELECT shelves.id, shelves.rack_id, shelves.label, shelves.height_mm,
                   racks.name AS rack_name
            FROM shelves
            JOIN racks ON racks.id = shelves.rack_id
            ORDER BY racks.name, shelves.label
        """)
    else:
        cursor.execute("""
            SELECT shelves.id, shelves.rack_id, shelves.label, shelves.height_mm,
                   racks.name AS rack_name
            FROM shelves
            JOIN racks ON racks.id = shelves.rack_id
            WHERE shelves.rack_id = ?
            ORDER BY shelves.label
        """, (rack_id,))
    rows = cursor.fetchall()
    connection.close()
    return [dict(row) for row in rows]


def get_shelf(shelf_id):
    connection = get_connection()
    cursor = connection.cursor()
    cursor.execute("""
        SELECT shelves.id, shelves.rack_id, shelves.label, shelves.height_mm,
               racks.name AS rack_name
        FROM shelves
        JOIN racks ON racks.id = shelves.rack_id
        WHERE shelves.id = ?
    """, (shelf_id,))
    row = cursor.fetchone()
    connection.close()
    return dict(row) if row else None


def create_order(shelf_id):
    connection = get_connection()
    cursor = connection.cursor()
    created_at = datetime.now().isoformat(timespec="seconds")
    cursor.execute(
        "INSERT INTO orders (shelf_id, status, created_at) VALUES (?, 'pending', ?)",
        (shelf_id, created_at),
    )
    connection.commit()
    order_id = cursor.lastrowid
    connection.close()
    return order_id


def list_orders():
    connection = get_connection()
    cursor = connection.cursor()
    cursor.execute("""
        SELECT orders.id, orders.status, orders.created_at, orders.completed_at,
               shelves.id AS shelf_id, shelves.label AS shelf_label, shelves.height_mm,
               racks.id AS rack_id, racks.name AS rack_name
        FROM orders
        JOIN shelves ON shelves.id = orders.shelf_id
        JOIN racks ON racks.id = shelves.rack_id
        ORDER BY orders.id DESC
    """)
    rows = cursor.fetchall()
    connection.close()
    return [dict(row) for row in rows]


def update_order_status(order_id, status):
    connection = get_connection()
    cursor = connection.cursor()
    if status == "completed":
        completed_at = datetime.now().isoformat(timespec="seconds")
        cursor.execute(
            "UPDATE orders SET status = ?, completed_at = ? WHERE id = ?",
            (status, completed_at, order_id),
        )
    else:
        cursor.execute(
            "UPDATE orders SET status = ? WHERE id = ?",
            (status, order_id),
        )
    connection.commit()
    connection.close()


def _print_table(title, rows):
    print(f"\n{title} ({len(rows)} rows)")
    if not rows:
        print("  (empty)")
        return
    columns = list(rows[0].keys())
    widths = {column: max(len(column), *(len(str(row[column])) for row in rows)) for column in columns}
    header = "  " + " | ".join(column.ljust(widths[column]) for column in columns)
    print(header)
    print("  " + "-" * (len(header) - 2))
    for row in rows:
        print("  " + " | ".join(str(row[column]).ljust(widths[column]) for column in columns))


if __name__ == "__main__":
    init_db()
    print(f"Database ready at: {DB_PATH}")
    _print_table("racks", list_racks())
    _print_table("shelves", list_shelves())
    _print_table("orders", list_orders())