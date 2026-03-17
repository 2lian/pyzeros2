import logging
from typing import Any, Optional
from uuid import uuid4

import ros_z_py

ZNode = Any
ZCtx = Any
ZCtxBuilder = Any

GLOBAL_BUILDER: Optional[ZCtxBuilder] = None
GLOBAL_CONTEXT: Optional[ZCtx] = None
GLOBAL_NODE: Optional[ZNode] = None

logger = logging.getLogger(__name__)


def set_auto_session(session: Optional[ZNode] = None) -> None:
    global GLOBAL_NODE
    GLOBAL_NODE = session


def auto_session(session: Optional[ZNode] = None) -> ZNode:
    global GLOBAL_BUILDER
    global GLOBAL_CONTEXT
    global GLOBAL_NODE
    if GLOBAL_NODE is not None:
        return GLOBAL_NODE
    if GLOBAL_CONTEXT is None:
        if GLOBAL_BUILDER is None:
            logger.info("Global ros_z_py builder: Creating")
            GLOBAL_BUILDER = ros_z_py.ZContextBuilder()
            logger.info("Global ros_z_py builder: Created")
        logger.info("Global ros_z_py context: Creating")
        GLOBAL_CONTEXT = GLOBAL_BUILDER.build()
        logger.info("Global ros_z_py context: Created")

    logger.info("Global ros_z_py session (node): Creating")
    ses = GLOBAL_CONTEXT.create_node(f"AFOR_{uuid4().hex}").build()
    set_auto_session(ses)
    logger.info("Global ros_z_py session (node): Created")
    return ses
