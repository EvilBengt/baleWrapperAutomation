Wiring
======


> **!OBS!**
> EJ aktuell/komplett!


Sladdar, ut
-----------

```JSON
{
    "-": "Jord",
    "11": null,
    "10": null,
    "9": "Räknare, (0V vid tillslag)",
    "8": "John Deere-ventil",
    "7": "Recirk-ventil",
    "6": "Sveparm, helfart",
    "5": "Sveparm, på",
    "4": "Breddning",
    "3": "Breddning",
    "2": "Kniv, stäng",
    "1": "Kniv, öppna"
}
```



Komponenter
-----------

```JSON
{
    "huvudbrytare": {
        "a": "12v, +",
        "b": [
            "reläkort [kniv, öppna] [no]",
            "reläkort [kniv, stäng] [no]",
            "reläkort [sveparm, på] [no]",
            "reläkort [sveparm, helfart] [no]",
            "reläkort [recirk-ventil] [no]",
            "reläkort [John Deere-ventil] [no]",
            "startknapp [in]",
            "on-relä [brytare] [b]",
            "on-knapp [a]"
        ]
    },
    "startknapp": {
        "in": "huvudbrytare [b]",
        "ut": "in-relä [start] [styrström]",
        "lampa": "12v [jord]"
    },
    "on-knapp": {
        "a": "huvudbrytare [b]",
        "b": [
            "on-relä [brytare] [b]",
            "on-relä [styrström] [a]"
        ]
    },
    "on-relä": {
        "styrström": {
            "a": [
                "on-knapp [b]",
                "on-relä [brytare] [b]"
            ],
            "b": "12v [jord]"
        },
        "brytare": {
            "a": "huvudbrytare [b]",
            "b": [
                "on-relä [styrström] [a]",
                "regulatorer [12v] [+]"
            ]
        },
        "hastighetsbegränsare": {
            "a": "huvudbrytare [b]",
            "b": "reläkort [rotor, helfart] [no]"
        }
    }
}
```



Reläkort
--------

```JSON
{
    "kniv, öppna": {
        "nc": "dosa [kniv, öppna (1)]",
        "com": "ut [kniv, öppna (1)]",
        "no": "huvudbrytare [b]"
    },
    "kniv, stäng": {
        "nc": "dosa [kniv, stäng (2)]",
        "com": "ut [kniv, stäng (2)]",
        "no": "huvudbrytare [b]"
    },
    "sveparm, på": {
        "nc": "dosa [sveparm, på (5)]",
        "com": "ut [sveparm, på (5)]",
        "no": "huvudbrytare [b]"
    },
    "sveparm, helfart": {
        "nc": "dosa [sveparm, helfart (6)]",
        "com": "ut [sveparm, helfart (6)]",
        "no": "huvudbrytare [b]"
    },
    "recirk-ventil": {
        "nc": "dosa [recirk-ventil (7)]",
        "com": "ut [recirk-ventil (7)]",
        "no": "huvudbrytare [b]"
    },
    "John Deere-ventil": {
        "nc": "dosa [John Deere-ventil (8)]",
        "com": "ut [John Deere-ventil (8)]",
        "no": "huvudbrytare [b]"
    },
    "räknare": {
        "nc": "dosa [räknare (9)]",
        "com": "ut [räknare (9)]",
        "no": "in-relä [räknare] [styrström]"
    },
    "?": {
        "nc": null,
        "com": null,
        "no": null
    }
}
```



In-relä
-------

```JSON
{
    "räknare": {
        "styrström": "reläkort [räknare] [no]"
    },
    "kniv, stängd": {
        "styrström": "ut [kniv stängd (X1)]"
    },
    "kniv, öppen": {
        "styrström": "ut [kniv öppen (X2)]"
    },
    "start": {
        "styrström": "startknapp [ut]"
    }
}
```
