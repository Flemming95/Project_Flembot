# Training Session 2: Relational Algebra Exercises

## Exercise 2.4.1: PC Database Queries

This exercise uses a database with the following relations:
- **Product**(maker, model, type)
- **PC**(model, speed, ram, hd, price)
- **Laptop**(model, speed, ram, hd, screen, price)
- **Printer**(model, color, type, price)

Write expressions of relational algebra to answer the following queries:

### a) What PC models have a speed of at least 3.00?

**Solution:**
```
π<sub>model</sub>(σ<sub>speed≥3.00</sub>(PC))
```

### b) Which manufacturers make laptops with a hard disk of at least 100GB?

**Solution:**
```
π<sub>maker</sub>(σ<sub>hd≥100</sub>(Laptop) ⨝ Product)
```

### c) Find the model number and price of all products (of any type) made by manufacturer B.

**Solution:**
```
π<sub>model,price</sub>(σ<sub>maker='B'</sub>(Product) ⨝ PC) ∪ 
π<sub>model,price</sub>(σ<sub>maker='B'</sub>(Product) ⨝ Laptop) ∪ 
π<sub>model,price</sub>(σ<sub>maker='B'</sub>(Product) ⨝ Printer)
```

### d) Find those manufacturers that sell Laptops, but not PC's.

**Solution:**
```
π<sub>maker</sub>(σ<sub>type='laptop'</sub>(Product)) − π<sub>maker</sub>(σ<sub>type='pc'</sub>(Product))
```

### e) Find those hard disk sizes that occur in two or more PC's.

**Solution:**
```
π<sub>hd</sub>(σ<sub>PC1.hd=PC2.hd AND PC1.model≠PC2.model</sub>(ρ<sub>PC1</sub>(PC) × ρ<sub>PC2</sub>(PC)))
```

### f) Find those pairs of PC models that have both the same speed and RAM. A pair should be listed only once; e.g., list (i,j) but not (j,i).

**Solution:**
```
π<sub>PC1.model,PC2.model</sub>(σ<sub>PC1.speed=PC2.speed AND PC1.ram=PC2.ram AND PC1.model<PC2.model</sub>(ρ<sub>PC1</sub>(PC) × ρ<sub>PC2</sub>(PC)))
```

### g) Find those manufacturers of at least two different computers (PC's or laptops) with speeds of at least 2.80.

**Solution:**
```
π<sub>maker</sub>(σ<sub>P1.maker=P2.maker AND P1.model≠P2.model</sub>(
  ρ<sub>P1</sub>(Product ⨝ (π<sub>model</sub>(σ<sub>speed≥2.80</sub>(PC)) ∪ π<sub>model</sub>(σ<sub>speed≥2.80</sub>(Laptop)))) × 
  ρ<sub>P2</sub>(Product ⨝ (π<sub>model</sub>(σ<sub>speed≥2.80</sub>(PC)) ∪ π<sub>model</sub>(σ<sub>speed≥2.80</sub>(Laptop))))
))
```

### h) Find those manufacturers of PC's that also make printers.

**Solution:**
```
π<sub>maker</sub>(σ<sub>type='pc'</sub>(Product)) ∩ π<sub>maker</sub>(σ<sub>type='printer'</sub>(Product))
```

---

## Exercise 2.4.2: Expression Trees

Below are the expression trees for the solutions in Exercise 2.4.1:

### a) Expression Tree: PC models with speed ≥ 3.00

```
        π (model)
           |
        σ (speed≥3.00)
           |
          PC
```

### b) Expression Tree: Manufacturers of laptops with HD ≥ 100GB

```
        π (maker)
           |
          ⨝
         / \
        /   \
       /     \
σ(hd≥100) Product
   |
 Laptop
```

### c) Expression Tree: Model and price of products by manufacturer B

```
                ∪
              /   \
             /     \
            ∪       π(model,price)
           / \          |
          /   \         ⨝
         /     \       / \
π(model,price) π(model,price)  σ(maker='B')  Printer
    |              |           /
    ⨝              ⨝      Product
   / \            / \
  /   \          /   \
σ(maker='B')  PC  σ(maker='B')  Laptop
   |                 |
Product          Product
```

### d) Expression Tree: Manufacturers selling laptops but not PCs

```
           −
          / \
         /   \
        /     \
  π(maker)    π(maker)
     |           |
σ(type='laptop') σ(type='pc')
     |           |
  Product     Product
```

### e) Expression Tree: Hard disk sizes in two or more PCs

```
           π (hd)
              |
σ (PC1.hd=PC2.hd AND PC1.model≠PC2.model)
              |
              ×
            /   \
           /     \
     ρ(PC1)(PC)  ρ(PC2)(PC)
```

### f) Expression Tree: PC pairs with same speed and RAM

```
           π (PC1.model, PC2.model)
                    |
σ (PC1.speed=PC2.speed AND PC1.ram=PC2.ram AND PC1.model<PC2.model)
                    |
                    ×
                  /   \
                 /     \
           ρ(PC1)(PC)  ρ(PC2)(PC)
```

### g) Expression Tree: Manufacturers with ≥2 computers with speed ≥ 2.80

```
                π (maker)
                    |
        σ (P1.maker=P2.maker AND P1.model≠P2.model)
                    |
                    ×
                  /   \
                 /     \
            ρ(P1)(...)  ρ(P2)(...)
                |           |
                ⨝           ⨝
              /   \       /   \
         Product   ∪   Product  ∪
                  / \           / \
                 /   \         /   \
         π(model)  π(model) π(model) π(model)
            |         |        |        |
      σ(speed≥2.80) σ(speed≥2.80) σ(speed≥2.80) σ(speed≥2.80)
            |         |        |        |
           PC      Laptop     PC     Laptop
```

### h) Expression Tree: Manufacturers of both PCs and printers

```
             ∩
            / \
           /   \
          /     \
     π(maker)   π(maker)
        |          |
  σ(type='pc') σ(type='printer')
        |          |
     Product    Product
```

---

## Exercise 2.4.3: World War II Capital Ships Database

This exercise uses a database with the following relations:

- **Classes**(class, type, country, numGuns, bore, displacement)
- **Ships**(name, class, launched)
- **Battles**(name, date)
- **Outcomes**(ship, battle, result)

### Sample Data

**Classes:**
| class | type | country | numGuns | bore | displacement |
|-------|------|---------|---------|------|--------------|
| Bismarck | bb | Germany | 8 | 15 | 42000 |
| Iowa | bb | USA | 9 | 16 | 46000 |
| Kongo | bc | Japan | 8 | 14 | 32000 |
| North Carolina | bb | USA | 9 | 16 | 37000 |
| Renown | bc | Gt. Britain | 6 | 15 | 32000 |
| Revenge | bb | Gt. Britain | 8 | 15 | 29000 |
| Tennessee | bb | USA | 12 | 14 | 32000 |
| Yamato | bb | Japan | 9 | 18 | 65000 |

**Ships:**
| name | class | launched |
|------|-------|----------|
| California | Tennessee | 1921 |
| Haruna | Kongo | 1915 |
| Hiei | Kongo | 1914 |
| Iowa | Iowa | 1943 |
| Kirishima | Kongo | 1915 |
| Kongo | Kongo | 1913 |
| Missouri | Iowa | 1944 |
| Musashi | Yamato | 1942 |
| New Jersey | Iowa | 1943 |
| North Carolina | North Carolina | 1941 |
| Ramillies | Revenge | 1917 |
| Renown | Renown | 1916 |
| Repulse | Renown | 1916 |
| Resolution | Revenge | 1916 |
| Revenge | Revenge | 1916 |
| Royal Oak | Revenge | 1916 |
| Royal Sovereign | Revenge | 1916 |
| Tennessee | Tennessee | 1920 |
| Washington | North Carolina | 1941 |
| Wisconsin | Iowa | 1944 |
| Yamato | Yamato | 1941 |

**Battles:**
| name | date |
|------|------|
| Denmark Strait | 1941-05-24 |
| Guadalcanal | 1942-11-15 |
| North Cape | 1943-12-26 |
| Surigao Strait | 1944-10-25 |

**Outcomes:**
| ship | battle | result |
|------|--------|--------|
| Bismarck | Denmark Strait | sunk |
| California | Surigao Strait | ok |
| Duke of York | North Cape | ok |
| Fuso | Surigao Strait | sunk |
| Hood | Denmark Strait | sunk |
| Kirishima | Guadalcanal | sunk |
| Prince of Wales | Denmark Strait | damaged |
| Rodney | Denmark Strait | ok |
| Scharnhorst | North Cape | sunk |
| South Dakota | Guadalcanal | damaged |
| Tennessee | Surigao Strait | ok |
| Washington | Guadalcanal | ok |
| West Virginia | Surigao Strait | ok |
| Yamashiro | Surigao Strait | sunk |

### Solutions

#### a) Give the class names and countries of the classes that carried guns of at least 16-inch bore.

**Solution:**
```
π<sub>class,country</sub>(σ<sub>bore≥16</sub>(Classes))
```

**Result on sample data:**
| class | country |
|-------|---------|
| Iowa | USA |
| North Carolina | USA |
| Yamato | Japan |

#### b) Find the ships launched prior to 1921.

**Solution:**
```
π<sub>name</sub>(σ<sub>launched<1921</sub>(Ships))
```

**Result on sample data:**
| name |
|------|
| Haruna |
| Hiei |
| Kirishima |
| Kongo |
| Ramillies |
| Renown |
| Repulse |
| Resolution |
| Revenge |
| Royal Oak |
| Royal Sovereign |
| Tennessee |

#### c) Find the ships sunk in the battle of the Denmark Strait.

**Solution:**
```
π<sub>ship</sub>(σ<sub>battle='Denmark Strait' AND result='sunk'</sub>(Outcomes))
```

**Result on sample data:**
| ship |
|------|
| Bismarck |
| Hood |

#### d) The treaty of Washington in 1921 prohibited capital ships heavier than 35,000 tons. List the ships that violated the treaty of Washington.

**Solution:**
```
π<sub>name</sub>(Ships ⨝ σ<sub>displacement>35000</sub>(Classes))
```

**Result on sample data:**
| name |
|------|
| Iowa |
| Missouri |
| Musashi |
| New Jersey |
| North Carolina |
| Washington |
| Wisconsin |
| Yamato |

#### e) List the name, displacement, and number of guns of the ships engaged in the battle of Guadalcanal.

**Solution:**
```
π<sub>name,displacement,numGuns</sub>(σ<sub>battle='Guadalcanal'</sub>(Outcomes) ⨝<sub>ship=name</sub> (Ships ⨝<sub>Ships.class=Classes.class</sub> Classes))
```

**Alternative solution with explicit renaming:**
```
π<sub>ship,displacement,numGuns</sub>(
  ρ<sub>ship/name</sub>((ρ<sub>name/ship</sub>(σ<sub>battle='Guadalcanal'</sub>(π<sub>ship</sub>(Outcomes))) 
   ⨝<sub>name=Ships.name</sub> Ships) 
  ⨝<sub>Ships.class=Classes.class</sub> Classes)
)
```

**Result on sample data:**
| name | displacement | numGuns |
|------|--------------|---------|
| Kirishima | 32000 | 8 |
| Washington | 37000 | 9 |

*Note: South Dakota appears in Outcomes for the Guadalcanal battle but does not appear in the Ships relation. Therefore, it cannot be joined with Classes and is not included in the result. Only ships that exist in the Ships relation can have their displacement and numGuns retrieved.*

#### f) List all the capital ships mentioned in the database. (Remember that all these ships may not appear in the Ships relation.)

**Solution:**
```
π<sub>name</sub>(Ships) ∪ ρ<sub>name/ship</sub>(π<sub>ship</sub>(Outcomes))
```

**Alternative solution (without explicit renaming):**
```
π<sub>name</sub>(Ships) ∪ π<sub>ship</sub>(Outcomes)
```

*Note: The primary solution uses renaming (ρ) to ensure consistent attribute names in the union result. While relational algebra may allow unions of relations with different attribute names when domains are compatible, using consistent naming is clearer and follows better practice.*

**Result on sample data:**
| name |
|------|
| Bismarck |
| California |
| Duke of York |
| Fuso |
| Haruna |
| Hiei |
| Hood |
| Iowa |
| Kirishima |
| Kongo |
| Missouri |
| Musashi |
| New Jersey |
| North Carolina |
| Prince of Wales |
| Ramillies |
| Renown |
| Repulse |
| Resolution |
| Revenge |
| Rodney |
| Royal Oak |
| Royal Sovereign |
| Scharnhorst |
| South Dakota |
| Tennessee |
| Washington |
| West Virginia |
| Wisconsin |
| Yamashiro |
| Yamato |

---

## Notes

- All relational algebra expressions use standard notation:
  - σ (sigma) for selection
  - π (pi) for projection
  - ⨝ for natural join
  - × for Cartesian product
  - ρ (rho) for renaming
  - ∪ for union
  - ∩ for intersection
  - − for set difference
  
- Subscripts are formatted using `<sub>` tags for proper display in markdown
- Expression trees show the hierarchical structure of relational algebra operations
