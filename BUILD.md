# 📋 Спецификация корабля (BOM)

Покупные компоненты для одного корабля (буксир). Состав платы — `Buksir v1.1`;
управление — драйвер <a href="https://github.com/ilya-koptev/ship-driver#readme" target="_blank" rel="noopener">ship-driver</a>.

> Количество выведено из схемы `Buksir v1.1`. Проверь позиции с разъёмами (DG308 / JST)
> и количество BLDC‑драйверов — взято по числу моторов.

| № | Компонент | Назначение | Ссылка | Кол‑во |
|---|---|---|---|---|
| 1 | **DC‑DC преобразователь** | Питание логики (step‑down, U3) | <a href="https://aliexpress.ru/item/1005009615339570.html" target="_blank" rel="noopener">купить</a> | 1 |
| 2 | **PWM8A04** | 3‑канальный RS485→PWM модуль 5 В, без пинов.(slave 11/12/13) | <a href="https://aliexpress.ru/item/1005004084317682.html" target="_blank" rel="noopener">купить</a> | 3 |
| 3 | **BLDC‑драйвер** | Драйвер ходовых моторов (U5) | <a href="https://aliexpress.ru/item/1005005372400680.html" target="_blank" rel="noopener">купить</a> | 4 |
| 4 | **DFPlayer mini** | MP3‑модуль, звук (U$1) | <a href="https://aliexpress.ru/item/1005009007157210.html" target="_blank" rel="noopener">купить</a> | 1 |
| 5 | **Разъёмы DG308** | Клеммные разъёмы (питание / шина) | <a href="https://aliexpress.ru/item/1005009534219319.html" target="_blank" rel="noopener">купить</a> | 4 |
| 6 | **Разъёмы JST** | Подключение двигателей (3 pin) (X3/X4/X6/X7) | <a href="https://aliexpress.ru/item/32875429112.html" target="_blank" rel="noopener">купить</a> | 4 |
| 7 | **DIP‑переключатель** | Конфигурация 1 юнит (S1) | <a href="https://aliexpress.ru/item/1005006743585812.html" target="_blank" rel="noopener">купить</a> | 1 |
| 8 | **Беспроводная зарядка** | Зарядка аккумулятора корабля | <a href="https://aliexpress.ru/item/1005003777900223.html" target="_blank" rel="noopener">купить</a> | 1 |
| 9 | **TTL → RS485** | Преобразователь шины (U1) | <a href="https://aliexpress.ru/item/32781637723.html" target="_blank" rel="noopener">купить</a> | 1 |
| 10 | **Электролитический конденсатор** | Фильтр питания (C2) | <a href="https://aliexpress.ru/item/1005005440501359.html" target="_blank" rel="noopener">купить</a> | 1 |
| 11 | **LoRa Ebyte E220‑900T22D** | Бортовой радиомодем (U2) | <a href="https://aliexpress.ru/item/1005002116186778.html" target="_blank" rel="noopener">купить</a> | 1 |
| 12 | **ULN2003A** | Драйвер света (7 ключей, OUT1–OUT5 → 5 каналов света) | — | 1 |

Большая часть есть на озоне.
Источник количеств: схема `Платка\buksir v1.1\Buksir v1.1.sch` (EAGLE).
Документация драйвера — [README](README.md).
