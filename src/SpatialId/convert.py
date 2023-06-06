# Copyright © 2022 Digital Agency. All rights reserved.
from tokenize import String


def in_to_internal(spatial_id: str) -> str:
    fields = spatial_id.split("/")
    out = "/".join([fields[0],fields[2],fields[3],fields[0],fields[1]])
    return out

def internal_to_out(spatial_id) -> str:
    if isinstance(spatial_id,str):
        fields = spatial_id.split("/")
        out = "/".join([fields[0],fields[4],fields[1],fields[2]])
        return out
    else:
        ret = []
        for s_id in spatial_id:
            ret.append(internal_to_out(s_id))
        return ret