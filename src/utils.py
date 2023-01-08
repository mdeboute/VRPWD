import pyproj
from Errors import ReferentialError

# ---------------------------------------------------
def project_coordinates(coordinates, initial_referential, output_referential):
    """Return a projected list of coordinates in a specified referential among 3857 (standard projection) or 4326 (GPS)"""
    assert isinstance(
        initial_referential, int
    ), "ERROR: initial_referential must be an integer!"
    assert isinstance(coordinates, list), "ERROR: coordinates must be a list!"
    assert isinstance(
        output_referential, int
    ), "ERROR: output_referential must be an integer!"
    round_number = 7
    list_referential = [3857, 4326]
    # check for correct referential
    if (initial_referential or output_referential) not in list_referential:
        raise ReferentialError(
            "ERROR: one or both referential are not correct - only 3857 and 4326 are allowed!"
        )
    list_projected_coords = []
    # create both projection
    initial_proj = pyproj.Proj(init="epsg:{}".format(initial_referential))
    output_proj = pyproj.Proj(init="epsg:{}".format(output_referential))
    # convert transformer instance
    transformer = pyproj.Transformer.from_proj(initial_proj, output_proj)
    for coord in coordinates:
        x, y = transformer.transform(coord[0], coord[1])
        x = round(x, round_number)
        y = round(y, round_number)
        list_projected_coords.append((x, y))
    return list_projected_coords
