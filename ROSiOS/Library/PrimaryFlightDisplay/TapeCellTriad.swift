//
//  TapeCellTriad.swift
//  PrimaryFlightDisplay
//
//  Created by Michael Koukoullis on 6/02/2016.
//  Copyright © 2016 Michael Koukoullis. All rights reserved.
//

typealias TapeCellStatus = (containsValue: Bool, cell: TapeCell)


class TapeCellTriad {
    
    fileprivate let cells: [TapeCell]
    
    init(cell1: TapeCell, cell2: TapeCell, cell3: TapeCell) {
        cells = [cell1, cell2, cell3]
    }
    
    func statusForValue(_ value: Double) -> (TapeCellStatus, TapeCellStatus, TapeCellStatus) {
        
        let statusCells = cells
            .sorted { $0.0.model.midValue < $0.1.model.midValue }
            .map  { ($0.model.containsValue(value), $0) }
        return (statusCells[0], statusCells[1], statusCells[2])
    }
}

extension TapeCellTriad: Sequence {
    
    func makeIterator() -> AnyIterator<TapeCell> {
        var nextIndex = 0
        
        return AnyIterator {
            guard nextIndex < self.cells.count else { return nil }
            
            let cell = self.cells[nextIndex]
            nextIndex += 1
            return cell
        }
    }
}
