import time
import zenoh
import asyncio_for_robotics.zenoh as afor


def main():
    with afor.auto_session() as session:
        replies = session.liveliness().declare_subscriber("@ros2_lv/**", history=True)
        for reply in replies:
            try:
                print(f"Alive token ('{reply.key_expr}')")
            except:
                print(f">> Received (ERROR: '{reply.payload.to_string()}')")


# --- Command line argument parsing --- --- --- --- --- ---
if __name__ == "__main__":
    main()

