/**
 * Простая feedforward нейросеть с 1 скрытым слоем
 * Архитектура: inputs -> hidden(8) -> output(1)
 * Активация: tanh
 */
export class NeuralNetwork {
  constructor(inputSize, hiddenSize = 8, outputSize = 1) {
    this.inputSize = inputSize;
    this.hiddenSize = hiddenSize;
    this.outputSize = outputSize;
    
    // Xavier инициализация весов
    this.weightsIH = this.randomMatrix(hiddenSize, inputSize, Math.sqrt(2 / inputSize));
    this.weightsHO = this.randomMatrix(outputSize, hiddenSize, Math.sqrt(2 / hiddenSize));
    this.biasH = new Array(hiddenSize).fill(0).map(() => (Math.random() - 0.5) * 0.5);
    this.biasO = new Array(outputSize).fill(0).map(() => (Math.random() - 0.5) * 0.5);
  }
  
  randomMatrix(rows, cols, scale) {
    return Array.from({ length: rows }, () =>
      Array.from({ length: cols }, () => (Math.random() - 0.5) * 2 * scale)
    );
  }
  
  forward(inputs) {
    const hidden = this.weightsIH.map((weights, i) => {
      let sum = this.biasH[i];
      for (let j = 0; j < inputs.length; j++) {
        sum += weights[j] * inputs[j];
      }
      return Math.tanh(sum);
    });
    
    const outputs = this.weightsHO.map((weights, i) => {
      let sum = this.biasO[i];
      for (let j = 0; j < hidden.length; j++) {
        sum += weights[j] * hidden[j];
      }
      return Math.tanh(sum);
    });
    
    return outputs;
  }
  
  toJSON() {
    return {
      inputSize: this.inputSize,
      hiddenSize: this.hiddenSize,
      outputSize: this.outputSize,
      weightsIH: this.weightsIH,
      weightsHO: this.weightsHO,
      biasH: this.biasH,
      biasO: this.biasO
    };
  }
  
  static fromJSON(data) {
    const nn = new NeuralNetwork(data.inputSize, data.hiddenSize, data.outputSize);
    nn.weightsIH = data.weightsIH;
    nn.weightsHO = data.weightsHO;
    nn.biasH = data.biasH;
    nn.biasO = data.biasO;
    return nn;
  }
  
  /**
   * Миграция сети на большее число входов.
   * Новые входы получают нулевые веса — сеть сохраняет старое поведение,
   * а новые входы обучаются с нуля (дообучение).
   */
  migrateInputSize(newInputSize) {
    if (newInputSize <= this.inputSize) return this;
    
    const newWeightsIH = this.weightsIH.map(row => {
      const newRow = [...row];
      while (newRow.length < newInputSize) {
        newRow.push(0);
      }
      return newRow;
    });
    
    const migrated = new NeuralNetwork(newInputSize, this.hiddenSize, this.outputSize);
    migrated.weightsIH = newWeightsIH;
    migrated.weightsHO = this.weightsHO;
    migrated.biasH = this.biasH;
    migrated.biasO = this.biasO;
    return migrated;
  }
  
  mutate(rate = 0.1, strength = 0.3) {
    const copy = new NeuralNetwork(this.inputSize, this.hiddenSize, this.outputSize);
    copy.weightsIH = this.mutateMatrix(this.weightsIH, rate, strength);
    copy.weightsHO = this.mutateMatrix(this.weightsHO, rate, strength);
    copy.biasH = this.mutateArray(this.biasH, rate, strength);
    copy.biasO = this.mutateArray(this.biasO, rate, strength);
    return copy;
  }
  
  mutateMatrix(matrix, rate, strength) {
    return matrix.map(row =>
      row.map(val => {
        if (Math.random() < rate) {
          return val + (Math.random() - 0.5) * strength;
        }
        return val;
      })
    );
  }
  
  mutateArray(arr, rate, strength) {
    return arr.map(val => {
      if (Math.random() < rate) {
        return val + (Math.random() - 0.5) * strength;
      }
      return val;
    });
  }
  
  static crossover(a, b) {
    const child = new NeuralNetwork(a.inputSize, a.hiddenSize, a.outputSize);
    child.weightsIH = NeuralNetwork.crossoverMatrix(a.weightsIH, b.weightsIH);
    child.weightsHO = NeuralNetwork.crossoverMatrix(a.weightsHO, b.weightsHO);
    child.biasH = NeuralNetwork.crossoverArray(a.biasH, b.biasH);
    child.biasO = NeuralNetwork.crossoverArray(a.biasO, b.biasO);
    return child;
  }
  
  static crossoverMatrix(a, b) {
    return a.map((row, i) =>
      row.map((val, j) => (Math.random() < 0.5 ? val : b[i][j]))
    );
  }
  
  static crossoverArray(a, b) {
    return a.map((val, i) => (Math.random() < 0.5 ? val : b[i]));
  }
}
