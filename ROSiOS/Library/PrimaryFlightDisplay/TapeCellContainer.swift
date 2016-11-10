//
//  TapeCellContainer.swift
//  PrimaryFlightDisplay
//
//  Created by Michael Koukoullis on 6/02/2016.
//  Copyright © 2016 Michael Koukoullis. All rights reserved.
//

import SpriteKit

class TapeCellContainer: SKNode {
    
    fileprivate let cellTriad: TapeCellTriad
    fileprivate let style: TapeIndicatorStyleType
    
    enum TapeCellError: Error {
        case SeedModelLowerValueMustBeZero(String)
    }
    
    init(seedModel: TapeCellModelType, style: TapeIndicatorStyleType) throws {
        let centerCell = TapeCell(model: seedModel, style: style)
        let previousCell = TapeCell(model: seedModel.previous(), style: style)
        let nextCell = TapeCell(model: seedModel.next(), style: style)
        cellTriad = TapeCellTriad(cell1: previousCell, cell2: centerCell, cell3: nextCell)
        self.style = style
        super.init()
        
        guard seedModel.lowerValue == 0 else {
            throw TapeCellError.SeedModelLowerValueMustBeZero("seed model lower value must be zero")
        }

        cellTriad.forEach { cell in
            addChild(cell)
            cell.position = cell.positionForZeroValue
        }
    }

    required init?(coder aDecoder: NSCoder) {
        fatalError("init(coder:) has not been implemented")
    }
    
    func actionForValue(_ value: Double) -> SKAction {
        switch style.type {
        case .continuous:
            return SKAction.move(to: positionForContinuousValue(value), duration: 0.05)
        case .compass:
            return SKAction.move(to: positionForCompassValue(value), duration: 0.2)
        }
    }
    
    func recycleCells() {
        let status = cellTriad.statusForValue(valueForPosition())
        
        switch status {
        case ((true, let cell1),  (false, _),  (false, let cell3)):
            recycleCell(cell3, model: cell1.model.previous())
            break
        case ((false, let cell1),  (false, _),  (true, let cell3)):
            recycleCell(cell1, model: cell3.model.next())
            break
        case ((false, let cell1),  (false, let cell2),  (false, let cell3)):
            let model = modelForValue(valueForPosition(), fromModel: cell2.model)
            recycleCell(cell1, model: model.previous())
            recycleCell(cell2, model: model)
            recycleCell(cell3, model: model.next())
            break
        default:
            break
        }
    }
    
    fileprivate func valueForPosition() -> Double {
        switch style.type {
        case .continuous:
            return continuousValueForPosition()
        case .compass:
            return continuousValueForPosition().compassValue
        }
    }
    
    fileprivate func positionForContinuousValue(_ value: Double) -> CGPoint {
        // TODO: Account for initial value
        let valuePosition =  -value * Double(style.pointsPerUnitValue)
        switch style.markerJustification {
        case .top, .bottom:
            return CGPoint(x: CGFloat(valuePosition), y: position.y)
        case .left, .right:
            return CGPoint(x: position.x, y: CGFloat(valuePosition))
        }
    }
    
    fileprivate func positionForCompassValue(_ compassValue: Double) -> CGPoint {
        let left = leftwardValueDeltaFromCompassValue(continuousValueForPosition().compassValue, toCompassValue: compassValue)
        let right = rightwardValueDeltaFromCompassValue(continuousValueForPosition().compassValue, toCompassValue: compassValue)
        
        if abs(left) < abs(right) {
            let newContinuousValue = continuousValueForPosition() + left
            return positionForContinuousValue(newContinuousValue)
        } else {
            let newContinuousValue = continuousValueForPosition() + right
            return positionForContinuousValue(newContinuousValue)
        }
    }
    
    fileprivate func continuousValueForPosition() -> Double {
        switch style.markerJustification {
        case .top, .bottom:
            return -Double(position.x) / Double(style.pointsPerUnitValue)
        case .left, .right:
            return -Double(position.y) / Double(style.pointsPerUnitValue)
        }
    }
    
    fileprivate func rightwardValueDeltaFromCompassValue(_ fromCompassValue: Double, toCompassValue: Double) -> Double {
        if fromCompassValue < toCompassValue {
            return toCompassValue - fromCompassValue
        }
        else {
            return toCompassValue - fromCompassValue + 360
        }
    }
    
    fileprivate func leftwardValueDeltaFromCompassValue(_ fromCompassValue: Double, toCompassValue: Double) -> Double {
        if fromCompassValue < toCompassValue {
            return toCompassValue - fromCompassValue - 360
        }
        else {
            return toCompassValue - fromCompassValue
        }
    }
    
    fileprivate func modelForValue(_ value: Double, fromModel model: TapeCellModelType) -> TapeCellModelType {
        if model.containsValue(value) {
            return model
        } else if value < model.midValue {
            return modelForValue(value, fromModel: model.previous())
        } else {
            return modelForValue(value, fromModel: model.next())
        }
    }
    
    fileprivate func recycleCell(_ cell: TapeCell, model: TapeCellModelType) {
        cell.model = model
        cell.position = cell.positionForZeroValue
    }
}
