from dataclasses import dataclass

@dataclass(frozen=True)
class Domain:
# espace de travail de l'effecteur du pantographe
    x_min: float = -0.20   # en mètres (ex: -20 cm)
    x_max: float =  0.20
    y_min: float =  0.00
    y_max: float =  0.30

DEFAULT_DOMAIN = Domain()

def is_in_domain(x: float, y: float, domain: Domain = DEFAULT_DOMAIN) -> bool:
    """Retourne True si (x,y) est dans la zone autorisée."""
    return (domain.x_min <= x <= domain.x_max) and (domain.y_min <= y <= domain.y_max)

def validate_domain(x: float, y: float, domain: Domain = DEFAULT_DOMAIN) -> None:
    """Lève une erreur si (x,y) est hors domaine (pratique pour sécuriser)."""
    if not is_in_domain(x, y, domain):
        raise ValueError(
            f"Point hors domaine: x={x}, y={y} "
            f"(attendu x∈[{domain.x_min},{domain.x_max}], y∈[{domain.y_min},{domain.y_max}])"
        )
