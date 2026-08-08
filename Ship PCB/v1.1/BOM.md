# 📋 Спецификация корабля (BOM) — Buksir v1.1

Покупные компоненты для одного корабля (буксир). Состав платы — `Buksir v1.1`;
управление — драйвер <a href="https://github.com/ilya-koptev/ship-driver#readme" target="_blank" rel="noopener">ship-driver</a>.

> Количество выведено из схемы `Buksir v1.1`. Проверь позиции с разъёмами (DG308 / JST)
> и количество BLDC‑драйверов — взято по числу моторов.

| № | Компонент | Назначение | Ссылка | Кол‑во |
|---|---|---|---|---|
| 1 | **DC‑DC преобразователь** | Питание логики (step‑down, U3) | <a href="https://aliexpress.ru/item/1005003502071127.html?sku_id=12000026080547853&spm=a2g2w.productlist.search_results.10.32da799eJx07VF">купить</a> | 1 |
| 2 | **PWM8A04** | 3‑канальный RS485→PWM модуль 5 В, без пинов.(slave 11/12/13) | <a href="https://aliexpress.ru/item/1005004084317682.html" target="_blank" rel="noopener">купить</a> | 3 |
| 3 | **BLDC‑драйвер** | Драйвер ходовых моторов (U5) | <a href="https://aliexpress.ru/item/1005005372400680.html" target="_blank" rel="noopener">купить</a> | 4 |
| 4 | **DFPlayer mini** | MP3‑модуль, звук (U$1) | <a href="https://aliexpress.ru/item/1005009007157210.html" target="_blank" rel="noopener">купить</a> | 1 |
| 5 | **Разъёмы DG308** | Клеммные разъёмы (питание / шина) | <a href="https://aliexpress.ru/item/1005009534219319.html" target="_blank" rel="noopener">купить</a> | 4 |
| 6 | **Разъёмы JST** | Подключение двигателей (3 pin) (X3/X4/X6/X7) | <a href="https://aliexpress.ru/item/32875429112.html" target="_blank" rel="noopener">купить</a> | 4 |
| 7 | **Беспроводная зарядка** | Зарядка аккумулятора корабля | <a href="https://aliexpress.ru/item/1005003777900223.html" target="_blank" rel="noopener">купить</a> | 1 |
| 8 | **TTL → RS485** | Преобразователь шины (U1) | <a href="https://aliexpress.ru/item/32781637723.html" target="_blank" rel="noopener">купить</a> | 1 |
| 9 | **Электролитический конденсатор** | Фильтр питания 1500 uF 16 V (C2) | <a href="https://aliexpress.ru/item/1005005440501359.html" target="_blank" rel="noopener">купить</a> | 1 |
| 10 | **LoRa Ebyte E220‑900T22D** | Бортовой радиомодем (U2) | <a href="https://aliexpress.ru/item/1005002116186778.html" target="_blank" rel="noopener">купить</a> | 1 |

Большая часть есть на озоне.
Источник количеств: схема `Buksir v1.1.sch` (EAGLE, в этой же папке).
Документация драйвера — <a href="https://github.com/ilya-koptev/ship-driver#readme" target="_blank" rel="noopener">README</a>.
